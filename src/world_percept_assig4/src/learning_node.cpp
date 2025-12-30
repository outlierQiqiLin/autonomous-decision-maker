#include <ros/ros.h>
#include <string>
#include <vector>
#include <iostream>
#include <cstdlib> 
#include <ctime>

#include <world_percept_assig4/GotoObject.h>
#include <world_percept_assig4/QueryKnowledge.h>

class LearningNode
{
private:
    ros::NodeHandle nh_;
    ros::ServiceClient client_reasoning_;
    ros::ServiceClient client_control_;

public:
    LearningNode(ros::NodeHandle& nh) : nh_(nh)
    {
        client_reasoning_ = nh_.serviceClient<world_percept_assig4::QueryKnowledge>("/query_knowledge");
        client_control_ = nh_.serviceClient<world_percept_assig4::GotoObject>("GotoObject");

        ROS_INFO("Learning Node initialized. Waiting for services...");
        ros::service::waitForService("/query_knowledge");
        ros::service::waitForService("GotoObject");
        ROS_INFO("Services connected.");
        
        // 随机数种子 (用于模拟随机探索)
        std::srand(std::time(0));
    }

    void run_mission(std::string target_object)
    {
        ROS_INFO_STREAM("=== MISSION START: Find [" << target_object << "] ===");

        // [Loop] 这是一个循环过程：感知 -> 思考 -> 行动 -> 再感知
        ros::Rate rate(0.5); // 2秒一次决策
        bool mission_complete = false;

        while(ros::ok() && !mission_complete)
        {
            // 1. 询问 Reasoning Node 候选地点 (FP.T01)
            world_percept_assig4::QueryKnowledge srv_know;
            srv_know.request.target_class = target_object;

            std::vector<std::string> candidates;
            if (client_reasoning_.call(srv_know))
            {
                candidates = srv_know.response.likely_locations;
            }
            else
            {
                ROS_ERROR("Failed to call Reasoning service.");
                break;
            }

            // 2. 决策阶段 (FP.T02 Learning)
            std::string action_target;
            
            if (candidates.empty())
            {
                // 情况 A: 没有任何线索 (比如刚启动，还没看见桌子)
                // 策略: 随机探索 / 原地旋转
                ROS_WARN("No candidates found. Exploring...");
                // 这里可以发指令让机器人去预设的 waypoint，或者只是等待 Percept Node 看到东西
                // 为了演示，我们暂时不做具体移动，只打印
                action_target = ""; 
            }
            else
            {
                // 情况 B: 有候选地点
                // 使用 ML 模型选择最佳地点
                action_target = selectBestLocation(candidates);
                ROS_INFO_STREAM("ML Model selected best target: " << action_target);
            }

            // 3. 执行阶段 (FP.T03 Robotics)
            if (!action_target.empty())
            {
                world_percept_assig4::GotoObject srv_ctrl;
                srv_ctrl.request.object_name = action_target;
                srv_ctrl.request.start = 1;

                if (client_control_.call(srv_ctrl))
                {
                    if (srv_ctrl.response.confirm) {
                        ROS_INFO("Robot moving...");
                        // 注意：这里应该有等待机制，等待机器人到达后再进行下一次循环
                        // 为了简化，我们假设 Control Node 会处理好
                        ros::Duration(15.0).sleep(); // 模拟移动耗时
                    }
                }
            }
            
            // 4. (可选) 检查任务是否完成
            // 可以在这里再次查询 Prolog: target_found(Target)?
            
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    // FP.T02: 机器学习/概率模型接口
    std::string selectBestLocation(const std::vector<std::string>& locations)
    {
        // 如果只有一个，直接去
        if (locations.size() == 1) return locations[0];

        std::string chosen = locations[0];
        double max_prob = -1.0;

        ROS_INFO("Evaluating candidates with Scikit-Learn (Simulated)...");

        for (const auto& loc : locations)
        {
            double prob = getProbabilityFromModel(loc);
            ROS_INFO_STREAM(" - " << loc << " : " << prob);
            
            if (prob > max_prob) {
                max_prob = prob;
                chosen = loc;
            }
        }
        return chosen;
    }

    // 模拟训练好的模型预测
    double getProbabilityFromModel(std::string loc)
    {
        // 简单逻辑：如果是 'table' 开头，概率高；如果是 'shelf'，概率低
        // 这里的逻辑就是 FP.T02 要求你“实现一种学习方法”的地方
        // 你可以把它改成读取 CSV 文件，或者调用 Python 脚本
        if (loc.find("table") != std::string::npos) return 0.8 + ((double)std::rand()/RAND_MAX)*0.1; 
        if (loc.find("shelf") != std::string::npos) return 0.3;
        return 0.1;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "learning_node");
    ros::NodeHandle nh;
    LearningNode learner(nh);

    ros::Duration(2.0).sleep(); 
    
    // 假设我们要找 bowl (对应 fp_reasoning.pl 里的逻辑)
    learner.run_mission("bowl"); 

    ros::spin();
    return 0;
}