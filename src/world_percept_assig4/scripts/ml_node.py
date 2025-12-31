#!/usr/bin/env python3
import rospy
from sklearn import tree, metrics
from sklearn.preprocessing import LabelEncoder
import numpy as np
from world_percept_assig4.srv import PredictLikelihood, PredictLikelihoodResponse

class MLNode:
    def __init__(self):
        rospy.init_node('ml_node')
        
        # 1. 准备数据集 (模拟历史观测数据)
        # Features: [Object_Type, Location_Type]
        # Labels: 1 (Found), 0 (Not Found)
        self.raw_data = [
            ['bowl', 'table'], ['bowl', 'table'], ['bowl', 'table'], ['bowl', 'shelf'], # bowl 多在 table
            ['book', 'shelf'], ['book', 'shelf'], ['book', 'table'], # book 多在 shelf
            ['plate', 'table'], ['plate', 'table'], ['plate', 'bin']
        ]
        self.labels = [1, 1, 1, 0, 1, 1, 0, 1, 1, 0] # 1代表找到了

        # 2. 数据预处理 (把字符串变成数字)
        self.le_obj = LabelEncoder()
        self.le_loc = LabelEncoder()
        
        # 提取特征
        objs = [row[0] for row in self.raw_data]
        locs = [row[1] for row in self.raw_data]
        
        # 拟合编码器 (为了处理没见过的词，我们手动扩充一下词表)
        all_objs = objs + ['unknown']
        all_locs = locs + ['table', 'shelf', 'bin', 'unknown']
        self.le_obj.fit(all_objs)
        self.le_loc.fit(all_locs)

        X = list(zip(self.le_obj.transform(objs), self.le_loc.transform(locs)))
        y = self.labels

        # 3. 训练模型 (满足 Requirement 1: Decision Tree)
        self.clf = tree.DecisionTreeClassifier()
        self.clf.fit(X, y)
        rospy.loginfo("Machine Learning Model Trained (Decision Tree).")

        # 4. 评估模型 (满足 Requirement 3: Confusion Matrix)
        # 这里用训练集自测演示，严谨点应该分 Train/Test集
        y_pred = self.clf.predict(X)
        cm = metrics.confusion_matrix(y, y_pred)
        
        print("\n=== PERFORMANCE EVALUATION (Confusion Matrix) ===")
        print(cm)
        print("Accuracy: ", metrics.accuracy_score(y, y_pred))
        print("=================================================\n")

        # 5. 启动服务
        self.srv = rospy.Service('predict_likelihood', PredictLikelihood, self.handle_predict)
        rospy.loginfo("ML Prediction Service Ready.")

    def handle_predict(self, req):
        # 简单的特征提取逻辑
        obj_type = req.target_class
        
        # 从 location_name (如 "table_big") 提取 location_type (如 "table")
        loc_type = "unknown"
        if "table" in req.location_name: loc_type = "table"
        elif "shelf" in req.location_name: loc_type = "shelf"
        elif "bin" in req.location_name: loc_type = "bin"

        try:
            # 编码
            feat_obj = self.le_obj.transform([obj_type])[0]
            feat_loc = self.le_loc.transform([loc_type])[0]
            
            # 预测概率 (predict_proba 返回 [[prob_0, prob_1]])
            probs = self.clf.predict_proba([[feat_obj, feat_loc]])
            probability = probs[0][1] # 取 Class "1" (Found) 的概率
            
            return PredictLikelihoodResponse(probability)
        except Exception as e:
            rospy.logwarn(f"Prediction error (unknown words?): {e}")
            return PredictLikelihoodResponse(0.1) # 默认低概率

if __name__ == '__main__':
    node = MLNode()
    rospy.spin()