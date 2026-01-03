// tiago_control_node.cpp
#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <Eigen/Dense>
#include <string>
#include <iostream>
#include <cmath>

// #include <world_percept_assig4/GetSceneObjectList.h>
#include <world_percept_assig4/GotoObject.h>

// Function to convert quaternion to 2D rotation matrix
Eigen::Matrix2d q2Rot2D(const geometry_msgs::Quaternion &q)
{
    Eigen::Quaterniond eigenQuaternion(q.w, q.x, q.y, q.z);
    Eigen::Matrix2d rotationMatrix = eigenQuaternion.toRotationMatrix().block(0, 0, 2, 2);
    return rotationMatrix;
}


class tiago_control_node
{
private:
    ros::NodeHandle nh_;

    // Subscriber to get the robot's pose from Gazebo
    ros::Subscriber model_states_sub_;

    // Publisher to send he twist information of the robot to the topic
    ros::Publisher  send_twist_pub_;

    // Service client to get target object information
    // ros::ServiceClient get_map_client_;

    // Service server to calculate the linear and angular velocity of the robot to move towards the target object
    ros::ServiceServer control_srv_;

    //current pose of the Tiago robot
    geometry_msgs::Pose tiago_pose_;
    bool has_tiago_pose_;

    //Target object pose
    geometry_msgs::Pose target_pose_;
    bool has_target_pose_;

    //start to move or not
    bool goto_object_;
    bool found_target;

    int call_interval_counter_ = 0;

    //target name
    std::string target_object_name_;

    // Navigation state
    enum NavState {
        IDLE,           
        MOVING,         
        BACKING_UP      
    };
    
    NavState nav_state_;
    ros::Time backup_start_time_;
    double backup_duration_;  // Backward motion duration (in seconds).

    //stop the robot
    void publishZeroV()
    {
        geometry_msgs::Twist cmd;
        cmd.linear.x=0.0;
        cmd.angular.z=0.0;
        send_twist_pub_.publish(cmd);
    }
    
    //call map_generator to get object pose
    // bool updateTargetPose(const std::string &object_name)
    // {
    //     world_percept_assig4::GetSceneObjectList srv;
    //     srv.request.object_name=object_name;

    //     if (!get_map_client_.call(srv))
    //     {
    //         ROS_ERROR("Failed to call get_map_client_");
    //         has_target_pose_=false;
    //         return false;
    //     }

    //     if (!srv.response.obj_found)
    //     {
    //         ROS_WARN("Object not found");
    //         has_target_pose_=false;
    //         return false;
    //     }

    //     //search exact name of object
    //     const gazebo_msgs::ModelStates &objs =srv.response.objects;

    //     bool matched = false;
    //     for (size_t i=0;i<objs.name.size(); ++i)
    //     {
    //         if(objs.name[i]==object_name)
    //         {
    //             target_pose_=objs.pose[i];
    //             matched = true;
    //             break;

    //         }
    //     }
    //     if (!matched)
    //     {ROS_WARN("Object found but pose entry missing.");
    //     has_target_pose_=false;
    //     return false;
        
    //     }
    //     has_target_pose_ = true;
    //     return true;
        
    // }

    //callback for gotoObject
    bool gotoObjectCallback(world_percept_assig4::GotoObject::Request &req,world_percept_assig4::GotoObject::Response &res)
    {
        // If `start` is equal to 2, it is interpreted as a request to enter the “query state.”
        // `confirm = true` indicates that the system is in an “idle / goal-reached” state, 
        //whereas `confirm = false` indicates that it is still in motion.
        if (req.start == 2)
        {
            // Arrival is considered valid only when the system is in the **IDLE** state.
            res.confirm = (nav_state_ == IDLE);
            
            if (nav_state_ == IDLE)
                res.message = "Idle/Arrived";
            else if (nav_state_ == MOVING)
                res.message = "Moving to target";
            else if (nav_state_ == BACKING_UP)
                res.message = "Backing up";

            return true;
        }

        if (req.start !=1)
        {
            res.confirm = false;
            res.message = "Invalid start value (expected 1 or 2)";
            return true;
        }
        //save target object name
        target_object_name_ = req.object_name;
        ROS_INFO("GotoObject request received.Target object = %s",target_object_name_.c_str());
        //get target pose once
        //if (!updateTargetPose(target_object_name_))
        //{
        //    res.confirm=false;
        //    res.message="failed to get target pose";
        //    return true;
        //}

        goto_object_ =true;
        has_target_pose_ = false;  
        nav_state_ = MOVING;  

        res.confirm =true;
        res.message ="Navigation started";
        return true;
    }
    void modelStateCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
    {
        call_interval_counter_++;
        //find tiago pose
        bool found_tiago =false;
        for (size_t i=0;i<msg->name.size();++i)
        {
            if (msg->name[i]=="tiago")
            {
                tiago_pose_=msg->pose[i];
                found_tiago=true;
            }
            if (goto_object_ && msg->name[i] == target_object_name_)
            {
                target_pose_ = msg->pose[i];
                has_target_pose_ = true;
            }
        }
        has_tiago_pose_= found_tiago;

        // Handle the backward state to prevent the robot from being too close to obstacles to safely reorient or turn.
        if (nav_state_ == BACKING_UP)
        {
            double elapsed = (ros::Time::now() - backup_start_time_).toSec();
            
            if (elapsed < backup_duration_)
            {
                // keep backward
                geometry_msgs::Twist cmd;
                cmd.linear.x = -0.2;  
                cmd.angular.z = 0.0;
                send_twist_pub_.publish(cmd);
                
                ROS_INFO_THROTTLE(0.5, "Backing up... %.1f/%.1f sec", 
                                elapsed, backup_duration_);
                return;
            }
            else
            {
                // backward achieved
                publishZeroV();
                ROS_INFO("Backup complete. Ready for next target.");
                
                nav_state_ = IDLE;
                goto_object_ = false;
                has_target_pose_ = false;
                return;
            }
        }

        // If the system is not in navigation mode, return immediately.
        if (!goto_object_)
            return;

        if (!has_tiago_pose_)
        {
            ROS_WARN_THROTTLE(2.0,"No tiago pose");
            return;
        }
        //  if(call_interval_counter_ % 50 == 0) 
        //   { 
        //      if(!updateTargetPose(target_object_name_)) 
        //      { 
        //          if(!has_target_pose_) 
            //      {    
            //        publishZeroV(); 
            //        return; 
            //       } 
        //      } 
        //   }

        if(!has_target_pose_) 
        { 
            ROS_WARN_THROTTLE(2.0, "Waiting for target pose: %s", target_object_name_.c_str());
            return; 
        }

        // Navigation logic.
        // Extract 2D positions
        Eigen::Vector2d tiago_w(tiago_pose_.position.x, tiago_pose_.position.y);
        Eigen::Vector2d target_w(target_pose_.position.x, target_pose_.position.y);
        // Calculate position difference in world frame
        Eigen::Vector2d Dpose_w = target_w - tiago_w;
        // Get 2D Rotation matrix from quaternion
        Eigen::Matrix2d Rtiago_w = q2Rot2D(tiago_pose_.orientation);
        // Calculate inverse rotation (world to tiago frame)
        Eigen::Matrix2d Rw_tiago = Rtiago_w.inverse();
        // Transform position to robot frame
        Eigen::Vector2d Dpose_tiago = Rw_tiago * Dpose_w;
        // Calculate distance
        double d = Dpose_tiago.norm(); //(Dpose_tiago.norm() <= 0.75) ? 0.0 : Dpose_tiago.norm();
        // Calculate angle to target
        double theta = std::atan2(Dpose_tiago(1), Dpose_tiago(0));
        // Control gains
        double Kwz = 1.5;
        double Kvx = 0.5;
        // Calculate velocities
        geometry_msgs::Twist tiago_twist_cmd;
        // tiago_twist_cmd.linear.x = Kvx * d;
        // tiago_twist_cmd.angular.z = Kwz * theta;
         
        
        if(d <= 1.0) 
        { 
            // Upon reaching the goal, switch to the backward state.
            publishZeroV();
            
            ROS_INFO("Target reached! Starting backup maneuver..."); 
            
            nav_state_ = BACKING_UP;
            backup_start_time_ = ros::Time::now();

            return;
        } 
        else 
        { 
            // normal movement
            nav_state_ = MOVING;
            tiago_twist_cmd.linear.x = Kvx * d; 
            tiago_twist_cmd.angular.z = Kwz * theta; 
            send_twist_pub_.publish(tiago_twist_cmd);
            ROS_INFO_THROTTLE(1.0, "Moving to target: distance = %.2f m", d);
        }

    }



public:
    tiago_control_node(ros::NodeHandle &nh)
        :nh_(nh),
        has_target_pose_(false),
        has_tiago_pose_(false),
        goto_object_(false),
        nav_state_(IDLE),          // initialize
        backup_duration_(5.0)      // backward for 5 seconds
    {
        model_states_sub_=nh_.subscribe("/gazebo/model_states", 10, &tiago_control_node::modelStateCallback, this);
        send_twist_pub_=nh_.advertise<geometry_msgs::Twist>("/key_vel", 10);
        // get_map_client_=nh_.serviceClient<world_percept_assig4::GetSceneObjectList>("get_scene_object_list");
        control_srv_ =nh.advertiseService("GotoObject",&tiago_control_node::gotoObjectCallback,this);
        ROS_INFO("tiago_control_node ready to go,use /GotoObject to begin navigation.");
    }   

};

int main(int argc, char **argv)
{   
    ros::init(argc, argv, "tiago_control_node");
    ros::NodeHandle nh;
    tiago_control_node node(nh);
    ros::spin();
    return 0;
}
