#include <ros/ros.h>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>

#include <rosprolog/rosprolog_client/PrologClient.h>
#include <world_percept_assig4/loadKnowledge.h>
using namespace std;

class Knowledge
{
private:
    PrologClient pl_;

    std::string srv_load_knowledge_name_;
    ros::ServiceServer load_knowledge_srv_;

    bool callback_load_knowledge(world_percept_assig4::loadKnowledge::Request &req,world_percept_assig4::loadKnowledge::Response &res);

    std::string query_file_name;

    void loadQueries();


public:
    Knowledge(ros::NodeHandle &nh)
    {
        ROS_INFO_STREAM("Wait for the Prolog service...");
        if(pl_.waitForServer())
        
            pl_ = PrologClient("/rosprolog", true);
        
        srv_load_knowledge_name_ = "load_knowledge";
        load_knowledge_srv_ = nh.advertiseService(srv_load_knowledge_name_ , &Knowledge::callback_load_knowledge, this);
        ROS_INFO_STREAM("Knowledge node:service'"<< srv_load_knowledge_name_ <<"'advertised.");

    }

    void setQueryFile (std::string fileName_Q)
    {
        std::ifstream file(fileName_Q.c_str());
        if (!file.is_open())
        {
            ROS_ERROR_STREAM("File not found and exit the function");
            return;
        }
        ROS_INFO_STREAM("File is successfully opened"<<fileName_Q);

        query_file_name=fileName_Q;
    }

};

void Knowledge::loadQueries()
{
    std::ifstream file(query_file_name.c_str());
    if(!file.is_open())
    {
        ROS_ERROR_STREAM("Failed to open query file:"<< query_file_name);
        return;
    }

    std::string line;
    while (std::getline(file,line))
    {
        if(line.empty())
        continue;
        ROS_INFO_STREAM("Sending query to Prolog: " << line);
        pl_.query(line);
    }
}

bool Knowledge::callback_load_knowledge(world_percept_assig4::loadKnowledge::Request &req,world_percept_assig4::loadKnowledge::Response &res)
{
    ROS_INFO_STREAM("load_knowledge service called, loading queries from: "
                    << query_file_name);
    loadQueries();
    res.confirm = true;
    ROS_INFO("Knowledge loading finished.");
    return true;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "knowledge_node");
    ros::NodeHandle nh;
    //expecting one value as an input argument
    if (argc<2)
    {
        ROS_ERROR_STREAM("Usage:knowledge_node<save_file_path>");
        return 1;
    }
    std::string saveFilePath;
    saveFilePath = argv[1];

    bool saveQueries_flag;
    if (!nh.getParam("read_prolog_queries/save_flag",saveQueries_flag))
   {
        ROS_WARN_STREAM("Parameter 'read_prolog_queries/save_flag' not found");
        saveQueries_flag=false;
   }
   std::string savedQueryFile;
   Knowledge myKnowledge(nh);
   if(saveQueries_flag)
   {
        if (!nh.getParam("read_prolog_queries/saved_query_file",savedQueryFile))
        {
            ROS_WARN_STREAM("Parameter 'read_prolog_queries/saved_query_file' not found");
            return 0;
        } 
        savedQueryFile= saveFilePath+savedQueryFile;
        ROS_INFO_STREAM("query_file: "<< savedQueryFile);
        
        myKnowledge.setQueryFile(savedQueryFile);
   }
   

   ros::spin();

   return 0;
            

}
