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

// Only UpdateObjectList is retained; this is the service type used by assert_knowledge.
#include <world_percept_assig4/UpdateObjectList.h>

class WorldInfo
{
private:
    std::string subs_topic_name_;        
    ros::Subscriber sub_gazebo_data_;    

    std::vector<std::string> v_seen_obj_;  // Records objects that have already been observed, in order to prevent redundant assertions.

    
    std::string srv_assert_knowledge_name_;
    ros::ServiceClient client_assert_knowledge_;

public:

    WorldInfo(ros::NodeHandle& nh)
    {
        ROS_INFO("Initializing Percept Node...");

        // Ignore list: the robot itself and the ground plane do not need to be stored in the knowledge base
        v_seen_obj_.push_back("tiago");
        v_seen_obj_.push_back("ground_plane");

        subs_topic_name_ = "/gazebo/model_states";

        // Initialize the Assert Knowledge client (connect to the Reasoning Node).
        srv_assert_knowledge_name_ = "/assert_knowledge";
        client_assert_knowledge_ = nh.serviceClient<world_percept_assig4::UpdateObjectList>(srv_assert_knowledge_name_);

        // Wait for the Reasoning Node to start up.
        ROS_INFO("Waiting for service %s ...", srv_assert_knowledge_name_.c_str());
        if(ros::service::waitForService(srv_assert_knowledge_name_, ros::Duration(30.0)))
        {
             ROS_INFO_STREAM("Connected to service: " << srv_assert_knowledge_name_);
        }
        else
        {
             ROS_ERROR("Reasoning Node service not found! Perception will not be saved.");
        }

        // Subscribe to Gazebo state updates to simulate perception.
        sub_gazebo_data_ = nh.subscribe(subs_topic_name_, 100, &WorldInfo::topic_callback, this);
    };

    ~WorldInfo() {};

private:

  void topic_callback(const gazebo_msgs::ModelStates::ConstPtr& msg)
  {
    // 1. Retrieve the robot's pose
    geometry_msgs::Pose tiago_pose;
    auto it = std::find(msg->name.begin(), msg->name.end(), "tiago");
    if (it != msg->name.end()) 
    {
        int index = std::distance(msg->name.begin(), it);
        tiago_pose = msg->pose.at(index);
    }
    else
    {
        // If the robot cannot be found, the distance cannot be computed; return immediately
        return; 
    }

    // 2. Iterate over all objects in the scene
    for (int i = 0; i < msg->name.size(); i++)
    {
        std::string obj_name = msg->name[i];
        geometry_msgs::Pose obj_pose = msg->pose[i];

        // caculate distances
        double dx = obj_pose.position.x - tiago_pose.position.x;
        double dy = obj_pose.position.y - tiago_pose.position.y;
        double d = std::sqrt(dx*dx + dy*dy);

        // if the object is within 1.5 meter
        if (d < 1.5)
        {
            // Check whether this object has already been processed.
            auto it_seen = std::find(v_seen_obj_.begin(), v_seen_obj_.end(), obj_name);

            // Not seen yet
            if (it_seen == v_seen_obj_.end()) 
            {
                ROS_INFO_STREAM("Percept: Saw new object [" << obj_name << "]");

                // Directly invoke the Reasoning Node’s service to perform the assertion.
                world_percept_assig4::UpdateObjectList srv;
                srv.request.object_name = obj_name;
                srv.request.object_pose = obj_pose;

                if (client_assert_knowledge_.call(srv))
                {
                    if (srv.response.confirmation)
                    {
                        ROS_INFO_STREAM(" -> Asserted [" << obj_name << "] into Knowledge Base.");
                        // Only add it to the “processed list” if the assertion succeeds
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