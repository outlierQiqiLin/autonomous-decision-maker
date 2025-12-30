#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ml_node.py —— Python 学习节点（scikit-learn: DecisionTreeClassifier）

提供服务：/ml/train_predict （world_percept_assig4/TrainPredict.srv）

功能：
  1) 读取本地 CSV 数据集（默认 ~/.ros/bowl_dataset.csv）
     每行样本格式（由 learning_node 写入）：
       visited_E,visited_NE,visited_N,visited_NW,visited_W,visited_SW,visited_S,visited_SE,y_sector
  2) 使用 DecisionTreeClassifier 训练分类器：根据 visited_* 预测 y_sector（8 分类）
  3) （评估）在日志里输出 accuracy + confusion matrix
  4) 对当前请求的特征向量做 predict_proba，返回 8 扇区概率

说明：
  - 数据量小的时候（<min_samples），返回均匀分布，保证系统可运行
  - 这是“直接可跑通”的实现；你之后可以把特征做得更合理（例如每个扇区看到的物体类别计数）
"""

import os
import rospy
import pandas as pd
import numpy as np

from world_percept_assig4.srv import TrainPredict, TrainPredictResponse

from sklearn.tree import DecisionTreeClassifier
from sklearn.model_selection import train_test_split
from sklearn.metrics import confusion_matrix, accuracy_score

SECTORS = ["E", "NE", "N", "NW", "W", "SW", "S", "SE"]

def expand_user(path: str) -> str:
    return os.path.expanduser(path)

def ensure_csv(path: str):
    if not os.path.exists(path):
        # 创建空 CSV（带表头）
        cols = [f"visited_{s}" for s in SECTORS] + ["y_sector"]
        pd.DataFrame(columns=cols).to_csv(path, index=False)

def load_dataset(csv_path: str) -> pd.DataFrame:
    ensure_csv(csv_path)
    df = pd.read_csv(csv_path)
    return df

def train_decision_tree(df: pd.DataFrame, max_depth: int = 4, min_samples: int = 10):
    """
    返回：clf 或 None（样本不足时）
    """
    if len(df) < min_samples:
        return None

    feature_cols = [f"visited_{s}" for s in SECTORS]
    X = df[feature_cols].fillna(0).astype(int).values
    y = df["y_sector"].astype(str).values

    # 如果类别太少，也可能学不出东西；但依然可以训练
    # 做一个简单 train/test split 评估（满足 Task2 的 evaluation 要求）
    try:
        X_tr, X_te, y_tr, y_te = train_test_split(
            X, y, test_size=0.2, random_state=0, stratify=y if len(set(y)) > 1 else None
        )
    except Exception:
        # 数据太少/类别问题导致 split 失败，就直接用全量训练并跳过评估
        X_tr, y_tr = X, y
        X_te, y_te = X, y

    clf = DecisionTreeClassifier(max_depth=max_depth, random_state=0)
    clf.fit(X_tr, y_tr)

    # 评估输出
    try:
        y_pred = clf.predict(X_te)
        acc = accuracy_score(y_te, y_pred)
        cm = confusion_matrix(y_te, y_pred, labels=SECTORS)
        rospy.loginfo(f"[ml] samples={len(df)} acc(test)={acc:.3f}")
        rospy.loginfo(f"[ml] confusion_matrix(labels={SECTORS})=\n{cm}")
    except Exception as e:
        rospy.logwarn(f"[ml] evaluation failed: {e}")

    # 为了预测稳定，我们再用全量数据拟合一次
    clf.fit(X, y)
    return clf

class MLNode:
    def __init__(self):
        self.csv_path = expand_user(rospy.get_param("~csv_path", "~/.ros/bowl_dataset.csv"))
        self.min_samples = int(rospy.get_param("~min_samples", 10))
        self.max_depth = int(rospy.get_param("~max_depth", 4))

        rospy.loginfo(f"[ml] csv_path={self.csv_path}, min_samples={self.min_samples}, max_depth={self.max_depth}")
        ensure_csv(self.csv_path)

        self.srv = rospy.Service("/ml/train_predict", TrainPredict, self.handle)

    def handle(self, req):
        df = load_dataset(self.csv_path)
        clf = train_decision_tree(df, max_depth=self.max_depth, min_samples=self.min_samples)

        # 当前请求的特征向量
        x = np.array([[
            int(req.visited_E), int(req.visited_NE), int(req.visited_N), int(req.visited_NW),
            int(req.visited_W), int(req.visited_SW), int(req.visited_S), int(req.visited_SE),
        ]], dtype=int)

        # 默认：均匀概率
        probs = {s: 1.0/len(SECTORS) for s in SECTORS}

        if clf is not None:
            # clf.classes_ 可能是 SECTORS 的子集，需要映射回完整 8 类
            try:
                p = clf.predict_proba(x)[0]
                for cls, pv in zip(clf.classes_, p):
                    probs[str(cls)] = float(pv)
            except Exception as e:
                rospy.logwarn(f"[ml] predict_proba failed: {e}")

        return TrainPredictResponse(
            p_E=probs["E"], p_NE=probs["NE"], p_N=probs["N"], p_NW=probs["NW"],
            p_W=probs["W"], p_SW=probs["SW"], p_S=probs["S"], p_SE=probs["SE"],
        )

if __name__ == "__main__":
    rospy.init_node("ml_node")
    MLNode()
    rospy.spin()
