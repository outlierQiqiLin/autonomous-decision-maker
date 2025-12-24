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
        
        std::srand(std::time(0));
    }

    // 辅助函数：发送通用查询并返回结果列表
    std::vector<std::string> askProlog(std::string query)
    {
        world_percept_assig4::QueryKnowledge srv;
        srv.request.target_class = query; // 发送完整查询语句
        
        if (client_reasoning_.call(srv))
        {
            return srv.response.likely_locations;
        }
        else
        {
            ROS_ERROR_STREAM("Failed to query: " << query);
            return std::vector<std::string>();
        }
    }

    // [核心]：可视化 Task 1 的完整 Pipeline
    void visualizePipeline(std::string target_obj)
    {
        ROS_INFO("================================================");
        ROS_INFO("       FINAL PROJECT PIPELINE VISUALIZATION     ");
        ROS_INFO("================================================");
        ROS_INFO("Target Goal: Find [%s]", target_obj.c_str());

        // ------------------------------------------------
        // 1. Perception Check (感知层)
        // ------------------------------------------------
        ROS_INFO("\n[Step 1] PERCEPTION CHECK (What has been seen?)");
        // 查询 seen(X)
        std::vector<std::string> seen_objs = askProlog("seen(X)");
        if (seen_objs.empty()) ROS_WARN(" -> Nothing seen yet.");
        for(auto s : seen_objs) ROS_INFO(" -> seen(%s)", s.c_str());

        // ------------------------------------------------
        // 2. Inference Check (推理层 - Task 1.1)
        // ------------------------------------------------
        ROS_INFO("\n[Step 2] SEMANTIC INFERENCE (Inferring properties)");
        
        // 测试 type 推断
        std::vector<std::string> types = askProlog("type(Obj, T)");
        for(auto t : types) ROS_INFO(" -> Infer Type: %s", t.c_str());

        // 测试 support_surface (桌子)
        std::vector<std::string> surfaces = askProlog("support_surface(X)");
        for(auto s : surfaces) ROS_INFO(" -> Infer SupportSurface: %s", s.c_str());

        // 测试 candidate_place (目标可能在哪)
        // 构造查询: candidate_place('plate', P)
        std::string cand_query = "candidate_place('" + target_obj + "', P)";
        std::vector<std::string> candidates = askProlog(cand_query);
        for(auto c : candidates) ROS_INFO(" -> Infer CandidatePlace: %s", c.c_str());

        // ------------------------------------------------
        // 3. Decision Check (决策层 - Task 1.3)
        // ------------------------------------------------
        ROS_INFO("\n[Step 3] DECISION MAKING (What to do next?)");

        // 测试 decide_search_order
        // 注意：Prolog 返回的是列表，这里简化打印
        std::string order_query = "decide_search_order('" + target_obj + "', List)";
        std::vector<std::string> order = askProlog(order_query);
        for(auto o : order) ROS_INFO(" -> Search Order List: %s", o.c_str());

        // 测试 decide_next_goal (当前要去哪)
        std::string goal_query = "decide_next_goal('" + target_obj + "', Place)";
        std::vector<std::string> next_goals = askProlog(goal_query);
        std::string current_goal = "";
        if (!next_goals.empty()) {
            current_goal = next_goals[0];
            ROS_INFO(" -> DECISION: Next Goal is [%s]", current_goal.c_str());
        } else {
            ROS_WARN(" -> DECISION: No goal (maybe explore?)");
        }

        // 测试 decide_action (最终高层动作)
        std::string action_query = "decide_action('" + target_obj + "', Action)";
        std::vector<std::string> actions = askProlog(action_query);
        for(auto a : actions) ROS_INFO(" -> FINAL ACTION: %s", a.c_str());

        ROS_INFO("================================================");

        // ------------------------------------------------
        // 4. Execution (执行层 - Task 3)
        // ------------------------------------------------
        if (!current_goal.empty() && current_goal != "none")
        {
            ROS_INFO("\n[Step 4] ROBOTIC EXECUTION");
            ROS_INFO("Sending command to Control Node: Go to [%s]", current_goal.c_str());
            
            world_percept_assig4::GotoObject srv_ctrl;
            srv_ctrl.request.object_name = current_goal;
            srv_ctrl.request.start = 1;

            if (client_control_.call(srv_ctrl)) {
                if (srv_ctrl.response.confirm) ROS_INFO(" -> Robot accepted command. Moving...");
            }
        }
    }

    void run_mission(std::string target_object)
    {
        // 设置初始任务状态 (Task 1 setup)
        askProlog("set_task(find_and_point('" + target_object + "'))");
        askProlog("set_home_pose(0,0,0)");

        ros::Rate rate(0.2); // 5秒一次循环，方便你看清楚输出
        while(ros::ok())
        {
            // 每次循环都运行一次可视化 Pipeline
            visualizePipeline(target_object);
            
            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "learning_node");
    ros::NodeHandle nh;
    LearningNode learner(nh);

    ros::Duration(2.0).sleep(); 
    
    // 假设我们要找 plate (对应 fp_reasoning.pl 里的逻辑)
    learner.run_mission("plate"); 

    ros::spin();
    return 0;
}