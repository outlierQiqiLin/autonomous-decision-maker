// learning_node.cpp
#include <ros/ros.h>
#include <string>
#include <vector>
#include <iostream>
#include <cstdlib> 
#include <ctime>

#include <world_percept_assig4/GotoObject.h>
#include <world_percept_assig4/QueryKnowledge.h>
#include <world_percept_assig4/PredictLikelihood.h>  
#include <world_percept_assig4/UpdateObjectList.h>  
#include <world_percept_assig4/AddObservation.h>

class LearningNode
{
private:
    ros::NodeHandle nh_;
    ros::ServiceClient client_reasoning_;
    ros::ServiceClient client_control_;
    ros::ServiceClient client_ml_;
    ros::ServiceClient client_update_;
    ros::ServiceClient client_ml_update_;


public:
    LearningNode(ros::NodeHandle& nh) : nh_(nh)
    {
        client_reasoning_ = nh_.serviceClient<world_percept_assig4::QueryKnowledge>("/query_knowledge");
        client_control_ = nh_.serviceClient<world_percept_assig4::GotoObject>("GotoObject");
        client_ml_ = nh_.serviceClient<world_percept_assig4::PredictLikelihood>("predict_likelihood");
        client_update_ = nh_.serviceClient<world_percept_assig4::UpdateObjectList>("/assert_knowledge");
        client_ml_update_ = nh_.serviceClient<world_percept_assig4::AddObservation>("add_observation");


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
                action_target = selectBestLocation(candidates, target_object);
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
                        ROS_INFO_STREAM("Command sent. Robot moving to " << action_target << "...");

                        bool arrived = false;
                        ros::Rate poll_rate(2); // 每秒检查2次
                        
                        // 设置一个最大超时时间，防止死循环 (比如60秒)
                        ros::Time start_time = ros::Time::now();
                        double timeout = 60.0;

                        while(ros::ok() && !arrived)
                        {
                            // 1. 发送查询请求 (start = 2)
                            world_percept_assig4::GotoObject srv_status;
                            srv_status.request.start = 2; 
                            srv_status.request.object_name = ""; // 不需要名字

                            if (client_control_.call(srv_status))
                            {
                                // 如果 confirm 为 true，说明 tiago_control_node 认为已经停止/到达
                                if (srv_status.response.confirm) {
                                    arrived = true;
                                    ROS_INFO("Robot arrived at target.");
                                } else {
                                    // 还在移动，继续等待
                                    ROS_INFO_THROTTLE(2.0, "Robot is still moving...");
                                }
                            }
                            else
                            {
                                ROS_WARN("Failed to call control service for status check.");
                            }

                            // 2. 检查超时
                            if ((ros::Time::now() - start_time).toSec() > timeout) {
                                ROS_WARN("Navigation Timeout! Robot took too long.");
                                break; // 强制跳出
                            }

                            ros::spinOnce();
                            poll_rate.sleep();
                        }

                        ROS_INFO("Scanning surroundings...");
                        ros::Duration(2.0).sleep();

                        // 1) 得到探索结果
                        bool found = checkTargetFound(target_object);

                        // 2) 回写给 Python 学习节点： (target_object, action_target, found)
                        world_percept_assig4::AddObservation srv_obs;
                        srv_obs.request.target_class   = target_object;
                        srv_obs.request.location_name  = action_target;
                        srv_obs.request.found          = found ? 1 : 0;

                        if (!client_ml_update_.call(srv_obs)) {
                            ROS_WARN("Failed to call add_observation (ML update).");
                        } else {
                            ROS_INFO_STREAM("ML updated: " << srv_obs.response.message);
                        }

                        // 3) 如果找到了，再结束 mission
                        if(found){
                            ROS_INFO("Mission accomplished, the target has been found!");
                            mission_complete = true;
                            break;
                        }

                        world_percept_assig4::UpdateObjectList srv_mark;
                        srv_mark.request.object_name = "visited:" + action_target;

                        srv_mark.request.object_pose.position.x = 0;
                        if(client_update_.call(srv_mark)) {
                            ROS_INFO_STREAM("SUCCESS: marked: " << action_target <<  " as visited in Prolog");
                        } else {
                            ROS_ERROR("Failed to update visited status.");
                        }
                    }
                }
            }
            
            
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    // FP.T02: 机器学习/概率模型接口
    std::string selectBestLocation(const std::vector<std::string>& locations, const std::string& current_target_object_)
    {
        // 如果只有一个，直接去
        if (locations.size() == 1) return locations[0];

        std::string chosen = locations[0];
        double max_prob = -1.0;

        ROS_INFO("Evaluating candidates with Scikit-Learn (Simulated)...");

        for (const auto& loc : locations)
        {
            double prob = getProbabilityFromModel(loc, current_target_object_);
            ROS_INFO_STREAM(" - " << loc << " : " << prob);
            
            if (prob > max_prob) {
                max_prob = prob;
                chosen = loc;
            }
        }
        return chosen;
    }

    // 模拟训练好的模型预测
    double getProbabilityFromModel(std::string loc, std::string target_obj)
    {
        // 这里的逻辑就是 FP.T02 要求“实现一种学习方法”的地方
        // 你可以把它改成读取 CSV 文件，或者调用 Python 脚本
        world_percept_assig4::PredictLikelihood srv;
        srv.request.target_class = target_obj;
        srv.request.location_name = loc;

        if (client_ml_.call(srv))
        {
            return (double)srv.response.probability;
        }
        else
        {
            ROS_WARN("Failed to call ML service, using default.");
            return 0.1; // Default fallback
        }
    }

    bool checkTargetFound(std::string target_class)
    {
        world_percept_assig4::QueryKnowledge srv;
        srv.request.target_class = "CHECK_FOUND:" + target_class;

        if (client_reasoning_.call(srv))
        {
            if (!srv.response.likely_locations.empty())
            {
                ROS_INFO_STREAM("Target found: " << srv.response.likely_locations[0] << " !");
                return true;
            }
        }
        return false;
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