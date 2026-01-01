#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <gazebo_msgs/ModelStates.h>
#include <cmath>
#include <vector>
#include <string>
#include <algorithm>

// [修改 1] 只保留 UpdateObjectList，这是 assert_knowledge 用的服务类型
#include <world_percept_assig4/UpdateObjectList.h>

class WorldInfo
{
private:
    std::string subs_topic_name_;        
    ros::Subscriber sub_gazebo_data_;    

    std::vector<std::string> v_seen_obj_;  // 记录已经看过的物体，防止重复 Assert

    // [修改 2] 只保留 assert_knowledge 的客户端
    std::string srv_assert_knowledge_name_;
    ros::ServiceClient client_assert_knowledge_;

public:

    WorldInfo(ros::NodeHandle& nh)
    {
        ROS_INFO("Initializing Percept Node...");

        // 忽略列表：机器人自己和地面不需要存入知识库
        v_seen_obj_.push_back("tiago");
        v_seen_obj_.push_back("ground_plane");

        subs_topic_name_ = "/gazebo/model_states";

        // [修改 3] 初始化 Assert Knowledge 客户端 (连接 Reasoning Node)
        srv_assert_knowledge_name_ = "/assert_knowledge";
        client_assert_knowledge_ = nh.serviceClient<world_percept_assig4::UpdateObjectList>(srv_assert_knowledge_name_);

        // 等待 Reasoning Node 启动
        ROS_INFO("Waiting for service %s ...", srv_assert_knowledge_name_.c_str());
        if(ros::service::waitForService(srv_assert_knowledge_name_, ros::Duration(30.0)))
        {
             ROS_INFO_STREAM("Connected to service: " << srv_assert_knowledge_name_);
        }
        else
        {
             ROS_ERROR("Reasoning Node service not found! Perception will not be saved.");
        }

        // 订阅 Gazebo 状态模拟视觉
        sub_gazebo_data_ = nh.subscribe(subs_topic_name_, 100, &WorldInfo::topic_callback, this);
    };

    ~WorldInfo() {};

private:

  void topic_callback(const gazebo_msgs::ModelStates::ConstPtr& msg)
  {
    // 1. 获取机器人位置
    geometry_msgs::Pose tiago_pose;
    auto it = std::find(msg->name.begin(), msg->name.end(), "tiago");
    if (it != msg->name.end()) 
    {
        int index = std::distance(msg->name.begin(), it);
        tiago_pose = msg->pose.at(index);
    }
    else
    {
        // 如果找不到机器人，无法计算距离，直接返回
        return; 
    }

    // 2. 遍历场景中的所有物体
    for (int i = 0; i < msg->name.size(); i++)
    {
        std::string obj_name = msg->name[i];
        geometry_msgs::Pose obj_pose = msg->pose[i];

        // 计算距离
        double dx = obj_pose.position.x - tiago_pose.position.x;
        double dy = obj_pose.position.y - tiago_pose.position.y;
        double d = std::sqrt(dx*dx + dy*dy);

        // [关键逻辑]：如果物体在 1 米以内（模拟摄像头视野）
        if (d < 1.5)
        {
            // 检查这个物体是否已经处理过了
            auto it_seen = std::find(v_seen_obj_.begin(), v_seen_obj_.end(), obj_name);

            // 如果是新物体 (Not seen yet)
            if (it_seen == v_seen_obj_.end()) 
            {
                ROS_INFO_STREAM("Percept: Saw new object [" << obj_name << "]");

                // [修改 4] 直接调用 Reasoning Node 的服务进行 Assert
                world_percept_assig4::UpdateObjectList srv;
                srv.request.object_name = obj_name;
                srv.request.object_pose = obj_pose;

                if (client_assert_knowledge_.call(srv))
                {
                    if (srv.response.confirmation)
                    {
                        ROS_INFO_STREAM(" -> Asserted [" << obj_name << "] into Knowledge Base.");
                        // 只有 Assert 成功了，才加入“已读列表”
                        v_seen_obj_.push_back(obj_name);
                    }
                    else
                    {
                        ROS_WARN(" -> Reasoning Node refused assertion.");
                    }
                }
                else
                {
                    ROS_ERROR_STREAM("Failed to call service " << srv_assert_knowledge_name_);
                }
            }
        } 
        else 
        {
            // [调试] 如果距离太远，打印出来看看（只打印一次，避免刷屏，可以用个简单的计数器或只打印特定物体）
            if (obj_name.find("bowl") != std::string::npos) {
                 ROS_WARN_STREAM_THROTTLE(2.0, "Percept: Bowl detected but too far! Dist=" << d);
            }
        }
    }
  } 
}; 

int main(int argc, char** argv)
{
    ros::init(argc, argv, "percept_node");
    ros::NodeHandle nh;
    WorldInfo myPercept(nh);
    ros::spin();
    return 0;
}