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
        
        // Random seed (used to simulate random exploration)
        std::srand(std::time(0));
    }

    void run_mission(std::string target_object)
    {
        ROS_INFO_STREAM("=== MISSION START: Find [" << target_object << "] ===");

        // [Loop] This is a cyclic process: perception -> reasoning -> action -> perception
        ros::Rate rate(0.5); // Decision is made every 2 seconds
        bool mission_complete = false;

        while(ros::ok() && !mission_complete)
        {
            // 1. Query the Reasoning Node for candidate locations (FP.T01)
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

            // 2. Decision-making phase (FP.T02 Learning)
            std::string action_target;
            
            if (candidates.empty())
            {
                // Case A: no available cues 
                // Strategy: random exploration / in-place rotation
                ROS_WARN("No candidates found. Exploring...");
                

                action_target = ""; 
            }
            else
            {
                // Case B: candidate locations are available.
                // Use the ML model to select the best location.
                action_target = selectBestLocation(candidates, target_object);
                ROS_INFO_STREAM("ML Model selected best target: " << action_target);
            }

            // 3. Execution phase
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
                        ros::Rate poll_rate(2); 
                        
                        
                        ros::Time start_time = ros::Time::now();
                        double timeout = 60.0;

                        while(ros::ok() && !arrived)
                        {
                            // 1. Send a query request (start = 2)
                            world_percept_assig4::GotoObject srv_status;
                            srv_status.request.start = 2; 
                            srv_status.request.object_name = ""; 

                            if (client_control_.call(srv_status))
                            {
                                // If confirm is true, it indicates that the tiago_control_node believes it has stopped / reached the target
                                if (srv_status.response.confirm) {
                                    arrived = true;
                                    ROS_INFO("Robot arrived at target.");
                                } else {
                                    // Still moving; continue waiting
                                    ROS_INFO_THROTTLE(2.0, "Robot is still moving...");
                                }
                            }
                            else
                            {
                                ROS_WARN("Failed to call control service for status check.");
                            }

                            // 2. Check for timeout
                            if ((ros::Time::now() - start_time).toSec() > timeout) {
                                ROS_WARN("Navigation Timeout! Robot took too long.");
                                break; 
                            }

                            ros::spinOnce();
                            poll_rate.sleep();
                        }

                        ROS_INFO("Scanning surroundings...");
                        ros::Duration(2.0).sleep();

                        // 1) Obtain the exploration results.
                        bool found = checkTargetFound(target_object);

                        // 2) Write back to the Python learning node: (target_object, action_target, found)
                        world_percept_assig4::AddObservation srv_obs;
                        srv_obs.request.target_class   = target_object;
                        srv_obs.request.location_name  = action_target;
                        srv_obs.request.found          = found ? 1 : 0;

                        if (!client_ml_update_.call(srv_obs)) {
                            ROS_WARN("Failed to call add_observation (ML update).");
                        } else {
                            ROS_INFO_STREAM("ML updated: " << srv_obs.response.message);
                        }

                        // 3) If the target is found, terminate the mission.
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
    // FP.T02: Machine learning / probabilistic model interface.
    std::string selectBestLocation(const std::vector<std::string>& locations, const std::string& current_target_object_)
    {
        // If there is only one option, go directly
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

    
    double getProbabilityFromModel(std::string loc, std::string target_obj)
    {
       

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
    
    // We aim to find the bowl (as defined by the logic in fp_reasoning.pl).
    learner.run_mission("bowl"); 

    ros::spin();
    return 0;
}