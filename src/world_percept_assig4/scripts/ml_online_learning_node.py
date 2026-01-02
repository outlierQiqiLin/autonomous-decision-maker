#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ml_online_learning_node.py
============================================================
【用途】
- 实现你描述的“探索-更新-再查询”的在线学习闭环：
  1) 初始先验：对任意(目标物体, 地点类别) 的“找到概率”默认 0.5
  2) 每次探索结束，外部节点把 (object, location, found) 喂给本节点
  3) 本节点累积成功/失败计数，并用带先验的方式更新概率
  4) 知识库在做 candidate 决策时向本节点查询概率，得到可用于排序的指导

【概率模型（可解释且稳定）】
- 对每个 (obj, loc_type) 维护 Beta-Bernoulli 计数：
    success = S, failure = F
- 先验设为 alpha=1, beta=1  => 初始均值 alpha/(alpha+beta)=0.5
- 每次观测：
    found=True  => S += 1
    found=False => F += 1
- 输出概率（后验均值）：
    p = (alpha + S) / (alpha + beta + S + F)

【ROS 接口（先做最简、方便你手动测试，后续好替换）】
- 使用 std_srvs/Trigger 做 service（避免你现在卡在自定义 .srv）
  1) /ml_node/add_observation  : 从 ROS 参数读取一次观测并更新模型
  2) /ml_node/predict          : 从 ROS 参数读取一次查询并返回概率
  3) /ml_node/rank_candidates  : 对固定四个候选地点输出概率（写入参数/发布topic）

【可视化概率变化】
- 每次 add_observation 后：
  - 在终端打印一个“简单条形图”（不用正规表格，但一眼能看变化）
  - 将每个候选地点的概率历史写入 CSV（默认 ~/.ros/ml_prob_history.csv）
  - 可选保存 plot 图片（默认 ~/.ros/ml_prob_plot.png），适合报告截图

【模块化接口预留】
- 你未来打通 pipeline 时，只需要把 handle_add_observation / handle_predict
  改成读取自定义 srv 的 req 并返回 res；底层 ProbabilityMemory 不用动。
"""

import os
import csv
import pickle
import threading
from dataclasses import dataclass, asdict
from typing import Dict, Tuple, List

from world_percept_assig4.srv import PredictLikelihood, PredictLikelihoodResponse
from world_percept_assig4.srv import AddObservation, AddObservationResponse


import rospy
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import String


# -----------------------------
# 1) 核心：概率记忆（与 ROS 解耦，后续替换接口不影响这里）
# -----------------------------
@dataclass
class BetaCounter:
    """某个 (obj, loc_type) 的成功/失败计数"""
    success: int = 0
    failure: int = 0


class ProbabilityMemory:
    """
    对 (object, location_type) 维护 Beta 计数，并输出后验概率均值。
    初始先验：alpha=beta=1 => p=0.5
    """
    def __init__(self, alpha: float = 1.0, beta: float = 1.0):
        self.alpha = float(alpha)
        self.beta = float(beta)
        self._mem: Dict[Tuple[str, str], BetaCounter] = {}
        self._lock = threading.Lock()

    @staticmethod
    def _norm(s: str) -> str:
        if s is None:
            return "unknown"
        return s.strip().lower()

    def update(self, obj: str, loc_type: str, found: bool) -> None:
        obj = self._norm(obj)
        loc_type = self._norm(loc_type)
        key = (obj, loc_type)
        with self._lock:
            if key not in self._mem:
                self._mem[key] = BetaCounter()
            if found:
                self._mem[key].success += 1
            else:
                self._mem[key].failure += 1

    def probability(self, obj: str, loc_type: str) -> float:
        obj = self._norm(obj)
        loc_type = self._norm(loc_type)
        key = (obj, loc_type)
        with self._lock:
            c = self._mem.get(key, BetaCounter())
            s, f = c.success, c.failure
        # 后验均值
        return float((self.alpha + s) / (self.alpha + self.beta + s + f))

    def snapshot_for_object(self, obj: str, loc_types: List[str]) -> List[Tuple[str, float, int, int]]:
        """返回指定 obj 在多个 loc_types 上的 (loc_type, p, S, F) 列表"""
        obj = self._norm(obj)
        out = []
        with self._lock:
            for lt in loc_types:
                lt2 = self._norm(lt)
                c = self._mem.get((obj, lt2), BetaCounter())
                p = (self.alpha + c.success) / (self.alpha + self.beta + c.success + c.failure)
                out.append((lt, float(p), c.success, c.failure))
        return out

    def dump_state(self) -> dict:
        """用于持久化：把 memory 转成可 pickle 的 dict"""
        with self._lock:
            mem_dump = { (k[0], k[1]) : asdict(v) for k, v in self._mem.items() }
        return {
            "alpha": self.alpha,
            "beta": self.beta,
            "mem": mem_dump
        }

    @staticmethod
    def load_state(state: dict) -> "ProbabilityMemory":
        pm = ProbabilityMemory(alpha=state.get("alpha", 1.0), beta=state.get("beta", 1.0))
        mem_dump = state.get("mem", {})
        with pm._lock:
            for (obj, lt), v in mem_dump.items():
                pm._mem[(obj, lt)] = BetaCounter(success=int(v.get("success", 0)), failure=int(v.get("failure", 0)))
        return pm


# -----------------------------
# 2) ROS 节点：最简 service + 可视化 + 持久化
# -----------------------------
class MLOnlineLearningNode:
    def __init__(self):
        rospy.init_node("ml_online_learning_node")

        # 四个候选地点（你当前场景固定这四个；未来可以改为参数或从 KB 传入）
        self.candidates_raw = [
            "book shelf",
            "cafe table",
            "table",
            "trash contain",
        ]

        # 先验：alpha=beta=1 => 初始 p=0.5
        self.alpha = float(rospy.get_param("~alpha", 1.0))
        self.beta = float(rospy.get_param("~beta", 1.0))

        # 持久化路径
        default_dir = os.path.join(os.path.expanduser("~"), ".ros")
        os.makedirs(default_dir, exist_ok=True)
        self.memory_path = rospy.get_param("~memory_path", os.path.join(default_dir, "ml_prob_memory.pkl"))
        self.csv_path = rospy.get_param("~csv_path", os.path.join(default_dir, "ml_prob_history.csv"))
        self.plot_path = rospy.get_param("~plot_path", os.path.join(default_dir, "ml_prob_plot.png"))

        # 是否每次更新都保存图片（需要 matplotlib）
        self.save_plot = bool(rospy.get_param("~save_plot", True))

        # 加载或初始化概率记忆
        self.pm = self._load_or_init_memory()
        # -----------------------------
        # 在线混淆矩阵（最简单版）
        # -----------------------------
        self.cm_threshold = 0.5   # p >= 0.5 认为“预测能找到”
        self.cm_tp = 0
        self.cm_fp = 0
        self.cm_tn = 0
        self.cm_fn = 0
        self.cm_n  = 0


        # 发布一个简单可读的 topic，方便你用 rostopic echo 看“概率快照”
        self.pub_snapshot = rospy.Publisher("/ml_node/prob_snapshot", String, queue_size=10)

        # 注册服务（最简 Trigger）
        self.srv_add = rospy.Service("/ml_node/add_observation", Trigger, self.handle_add_observation)
        self.srv_pred = rospy.Service("/ml_node/predict", Trigger, self.handle_predict)
        self.srv_rank = rospy.Service("/ml_node/rank_candidates", Trigger, self.handle_rank_candidates)

        # ★给 C++ learning_node 用的：概率查询服务（名字必须是 predict_likelihood）
        self.srv_pred_cpp = rospy.Service("predict_likelihood", PredictLikelihood, self.handle_predict_srv)

        # ★给 C++ learning_node 用的：探索结果写入服务（名字必须是 add_observation）
        self.srv_add_cpp = rospy.Service("add_observation", AddObservation, self.handle_add_observation_srv)
        rospy.loginfo("ML online node start.")
        rospy.loginfo("Service:/ml_node/add_observation  /ml_node/predict  /ml_node/rank_candidates")
        rospy.loginfo("memorey files:%s", self.memory_path)
        rospy.loginfo("history CSV:%s", self.csv_path)
        rospy.loginfo("plot :%s (save_plot=%s)", self.plot_path, self.save_plot)

        # 启动时也发一次默认快照（方便你立刻看到初始 0.5）
        self._publish_and_log_snapshot(obj="bowl")
    # -----------------------------
    # 4) C++ 对接接口：srv 版本（推荐正式 pipeline 使用）
    # -----------------------------
    def handle_predict_srv(self, req):
        """
        C++ learning_node 会调用的概率查询接口：
        请求：target_class, location_name
        响应：probability
        """
        obj = req.target_class
        loc_name = req.location_name
        loc_type = self.normalize_location(loc_name)

        p = self.pm.probability(obj, loc_type)
        return PredictLikelihoodResponse(p)
    
    def _update_and_print_confusion(self, p_before: float, found: bool):
    
        y_pred = 1 if p_before >= self.cm_threshold else 0
        y_true = 1 if found else 0

        if y_pred == 1 and y_true == 1:
            self.cm_tp += 1
        elif y_pred == 1 and y_true == 0:
            self.cm_fp += 1
        elif y_pred == 0 and y_true == 0:
            self.cm_tn += 1
        else:
            self.cm_fn += 1

        self.cm_n += 1

        
        acc = (self.cm_tp + self.cm_tn) / self.cm_n if self.cm_n > 0 else 0.0

        print("=== Confusion Matrix (online) ===")
        print("threshold =", self.cm_threshold)
        print("Pred\\True | found(1) | not_found(0)")
        print(f"found(1)  | TP={self.cm_tp:<4d} | FP={self.cm_fp:<4d}")
        print(f"not(0)    | FN={self.cm_fn:<4d} | TN={self.cm_tn:<4d}")
        print(f"n={self.cm_n}  accuracy={acc:.3f}")
        print()



    def handle_add_observation_srv(self, req):
        """
        C++ learning_node 会调用的写入探索结果接口：
        请求：target_class, location_name, found
        响应：ok, message
        """
        obj = req.target_class
        loc_name = req.location_name
        found = bool(req.found)

        loc_type = self.normalize_location(loc_name)

        # ★ 1) 更新前先取一次概率，用于混淆矩阵
        p_before = self.pm.probability(obj, loc_type)
        self._update_and_print_confusion(p_before, found)

        # ★ 2) 再进行真正的学习更新
        self.pm.update(obj, loc_type, found)

        ok, save_msg = self._save_memory()

        # ★ 3) 原有的概率柱状图（保持不变）
        self._publish_and_log_snapshot(obj=obj)


        msg = f"updated: obj={obj}, loc_name='{loc_name}' -> loc_type={loc_type}, found={found}; {save_msg}"
        rospy.loginfo(msg)

        return AddObservationResponse(ok, msg)





    # ---------- 地点名归一化（你以后根据 KB 命名随时扩展这里） ----------
    def normalize_location(self, location_name: str) -> str:
        """
        将任意 location_name 归一化到 4 类（或 unknown）：
        - book_shelf
        - cafe_table
        - table
        - trash_container
        """
        if location_name is None:
            return "unknown"
        s = location_name.strip().lower().replace("-", "_").replace(" ", "_")

        if "book" in s and "shelf" in s:
            return "book_shelf"
        if "cafe" in s and "table" in s:
            return "cafe_table"
        # 注意：cafe_table 要先匹配，否则会被下面 table 吃掉
        if "table" in s:
            return "table"
        if "trash" in s and ("contain" in s or "container" in s or "bin" in s):
            return "trash_container"
        if "trash_container" in s:
            return "trash_container"
        if "bin" in s:
            return "trash_container"

        return "unknown"

    def candidate_loc_types(self) -> List[str]:
        """把四个候选地点映射到归一化类别"""
        return [self.normalize_location(x) for x in self.candidates_raw]

    # ---------- 持久化 ----------
    def _load_or_init_memory(self) -> ProbabilityMemory:
        if os.path.exists(self.memory_path):
            try:
                with open(self.memory_path, "rb") as f:
                    state = pickle.load(f)
                pm = ProbabilityMemory.load_state(state)
                rospy.loginfo("loading history of probability")
                return pm
            except Exception as e:
                rospy.logwarn("加载记忆失败，将重新初始化。原因：%s", str(e))
        return ProbabilityMemory(alpha=self.alpha, beta=self.beta)

    def _save_memory(self) -> Tuple[bool, str]:
        try:
            with open(self.memory_path, "wb") as f:
                pickle.dump(self.pm.dump_state(), f)
            return True, "保存成功"
        except Exception as e:
            return False, f"保存失败：{e}"

    # ---------- 简单可视化：条形图（终端） ----------
    @staticmethod
    def _bar(p: float, width: int = 20) -> str:
        p = max(0.0, min(1.0, float(p)))
        n = int(round(p * width))
        return "█" * n + "░" * (width - n)

    def _publish_and_log_snapshot(self, obj: str) -> None:
        """
        生成四候选地点概率快照：
        - 发布到 /ml_node/prob_snapshot (String)
        - 控制台打印一段“直观条形图”
        - 写入 CSV 并可选保存图片
        """
        loc_types = self.candidate_loc_types()
        snap = self.pm.snapshot_for_object(obj, loc_types)

        # 排序（高到低）
        snap_sorted = sorted(snap, key=lambda x: x[1], reverse=True)

        # 组织字符串：便于 rostopic echo
        lines = [f"Target  object: {obj}  (Prio alpha=beta=1 => init 0.5)"]
        for lt, p, s, f in snap_sorted:
            lines.append(f"- {lt:<15s}  p={p:.4f}  S={s} F={f}  {self._bar(p)}")
        best = snap_sorted[0][0]
        msg = "\n".join(lines)

        # 发布 topic
        self.pub_snapshot.publish(String(data=msg))

        # 终端打印（你说不需要正规表格，这里只打印条形图+数字）
        print("\n" + msg + "\n")

        # 记录到 CSV（用于“可视化变化”——你可以用 Excel/Matplotlib/gnuplot）
        self._append_csv(obj, snap)

        # 可选保存图片（更直观，适合报告截图）
        if self.save_plot:
            self._save_plot_png(obj)

    def _append_csv(self, obj: str, snap: List[Tuple[str, float, int, int]]) -> None:
        """
        CSV 列：
        timestamp, object, loc_type, prob, success, failure
        """
        import datetime
        ts = datetime.datetime.now().isoformat(timespec="seconds")

        file_exists = os.path.exists(self.csv_path)
        try:
            with open(self.csv_path, "a", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                if not file_exists:
                    w.writerow(["timestamp", "object", "loc_type", "prob", "success", "failure"])
                for lt, p, s, ff in snap:
                    w.writerow([ts, obj, lt, f"{p:.6f}", s, ff])
        except Exception as e:
            rospy.logwarn("写CSV失败：%s", str(e))

    def _save_plot_png(self, obj: str) -> None:
        """
        保存一个简单柱状图到 plot_path。
        注意：如果你的环境没装 matplotlib，把 ~save_plot 设为 false 即可。
        """
        try:
            import matplotlib
            matplotlib.use("Agg")  # 无界面保存
            import matplotlib.pyplot as plt
        except Exception as e:
            rospy.logwarn("matplotlib 不可用，无法保存图片。可将 ~save_plot:=false。原因：%s", str(e))
            return

        loc_types = self.candidate_loc_types()
        snap = self.pm.snapshot_for_object(obj, loc_types)
        snap_sorted = sorted(snap, key=lambda x: x[1], reverse=True)

        labels = [x[0] for x in snap_sorted]
        probs = [x[1] for x in snap_sorted]

        plt.figure()
        plt.title(f"Probability of finding '{obj}' by location (Beta prior mean)")
        plt.xlabel("location_type")
        plt.ylabel("P(found)")
        plt.ylim(0.0, 1.0)
        plt.bar(labels, probs)
        plt.tight_layout()
        try:
            plt.savefig(self.plot_path, dpi=160)
        except Exception as e:
            rospy.logwarn("保存图片失败：%s", str(e))
        finally:
            plt.close()

    # -----------------------------
    # 3) ROS Service 回调：最简 Trigger + 参数传参
    # -----------------------------
    def handle_add_observation(self, _req):
        """
        从参数读取一条观测并更新：
          ~obs_target_class  (str)  例如 "bowl"
          ~obs_location_name (str)  例如 "cafe table" / "book shelf" / ...
          ~obs_found         (bool) True/False
        """
        obj = rospy.get_param("~obs_target_class", "bowl")
        loc_name = rospy.get_param("~obs_location_name", "table")
        found = bool(rospy.get_param("~obs_found", False))

        loc_type = self.normalize_location(loc_name)

        # 更新概率记忆
        self.pm.update(obj, loc_type, found)

        ok, save_msg = self._save_memory()

        # 更新后立即发布/展示一次快照（你要的“概率变化可视化”）
        self._publish_and_log_snapshot(obj=obj)

        msg = f"已更新观测：obj={obj}, loc_name='{loc_name}' -> loc_type={loc_type}, found={found}；{save_msg}"
        rospy.loginfo(msg)

        return TriggerResponse(success=True if ok else False, message=msg)

    def handle_predict(self, _req):
        """
        从参数读取一次查询并返回概率：
          ~query_target_class  (str)
          ~query_location_name (str)
        """
        obj = rospy.get_param("~query_target_class", "bowl")
        loc_name = rospy.get_param("~query_location_name", "table")
        loc_type = self.normalize_location(loc_name)

        p = self.pm.probability(obj, loc_type)

        # 也写到参数，方便别的模块临时读取
        rospy.set_param("~last_probability", float(p))
        rospy.set_param("~last_query_loc_type", loc_type)

        msg = f"P(found=1 | obj={obj}, loc_type={loc_type}) = {p:.4f}"
        return TriggerResponse(success=True, message=msg)

    def handle_rank_candidates(self, _req):
        """
        对四个候选地点输出概率（用于 KB 做 candidate 排序）：
          输入参数：~rank_target_class (str)
          输出：
            - TriggerResponse.message 给出 best location_type
            - 参数 ~rank_result  写入结构化列表，便于上游模块读取
            - topic /ml_node/prob_snapshot 同步发布快照
        """
        obj = rospy.get_param("~rank_target_class", "bowl")
        loc_types = self.candidate_loc_types()
        snap = self.pm.snapshot_for_object(obj, loc_types)
        snap_sorted = sorted(snap, key=lambda x: x[1], reverse=True)

        best_loc_type, best_p, _, _ = snap_sorted[0]

        # 写参数（结构化输出）
        rank_result = [{"loc_type": lt, "prob": float(p), "success": int(s), "failure": int(f)} for lt, p, s, f in snap_sorted]
        rospy.set_param("~rank_result", rank_result)
        rospy.set_param("~rank_best_loc_type", best_loc_type)
        rospy.set_param("~rank_best_prob", float(best_p))

        # 同步发布/打印快照（可视化变化）
        self._publish_and_log_snapshot(obj=obj)

        msg = f"best={best_loc_type}, prob={best_p:.4f}"
        return TriggerResponse(success=True, message=msg)


if __name__ == "__main__":
    node = MLOnlineLearningNode()
    rospy.spin()
