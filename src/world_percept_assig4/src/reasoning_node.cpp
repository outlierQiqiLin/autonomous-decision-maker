#include <ros/ros.h>
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <vector>

#include <rosprolog/rosprolog_client/PrologClient.h>

// 引入服务头文件
#include <world_percept_assig4/UpdateObjectList.h>
#include <world_percept_assig4/QueryKnowledge.h>

using namespace std;

class Reasoner
{
private: 
    PrologClient pl_;
    
    ros::ServiceServer assert_knowledge_srv_; 
    ros::ServiceServer query_knowledge_srv_;  

    // 日志相关 (保留)
    bool m_query_flag_save;
    std::string m_query_file_name;

public:

    Reasoner(ros::NodeHandle &nh)
    {
        ROS_INFO_STREAM("Wait for the Prolog service...");
        if(pl_.waitForServer())
            pl_ = PrologClient("/rosprolog", true);

        // 初始化服务
        assert_knowledge_srv_ = nh.advertiseService("/assert_knowledge", &Reasoner::srv_assert_callback, this);
        query_knowledge_srv_ = nh.advertiseService("/query_knowledge", &Reasoner::srv_query_callback, this);

        this->m_query_flag_save=false;
        
        // --- 初始化 KB (FP.T01) ---
        // 每次节点启动时，重置知识库，保证状态干净
        ROS_INFO("Initializing Knowledge Base...");
        pl_.query("reset_kb");
        pl_.query("set_home_pose(0,0,0)");
    };

    ~Reasoner(){};

    void setOutQueriesFile(string QueryfileName)
    {
        std::ofstream file(QueryfileName.c_str(),std::ios::app);
        if (!file.is_open()) {
            ROS_ERROR_STREAM("File or directory is not found"<<QueryfileName);
            this->m_query_flag_save = false;
        } else {
            ROS_INFO_STREAM("File is successfully opened"<<QueryfileName);
            this->m_query_flag_save = true;  
            this->m_query_file_name = QueryfileName; 
        }
    }

private:    

    // ---------------------------------------------------------
    // Callback: 存知识 (Percept -> Reasoning)
    // ---------------------------------------------------------
    bool srv_assert_callback(world_percept_assig4::UpdateObjectList::Request &req,
                             world_percept_assig4::UpdateObjectList::Response &res)
    {
        if(req.object_name.find("visited:") == 0){
            std::string place_name = req.object_name.substr(8);
            ROS_INFO_STREAM("Reasoning: marking [" << place_name << "] as visited");

            std::stringstream ss;
            ss << "assertz(visited('" << place_name << "'))";

            try {
                PrologQuery bdgs = pl_.query(ss.str());
                for(auto const& solution : bdgs) { break; }
                res.confirmation = true;

            } catch (exception &e) {
                res.confirmation = false;
            }
            return true;


        }
        ROS_INFO_STREAM("Reasoning: Asserting perception of [" << req.object_name << "]");
        
        double x = req.object_pose.position.x;
        double y = req.object_pose.position.y;
        double z = req.object_pose.position.z;

        // 构造 Prolog 查询: assert_percept('name', x, y, z).
        // 注意：使用单引号包围名字，防止 Prolog 把小写字母开头的名字当成 atom，大写当成变量
        std::stringstream ss;
        ss << "assert_percept('" << req.object_name << "', " << x << ", " << y << ", " << z << ")";
        
        std::string query = ss.str();
        
        // 执行 Prolog 查询 (assert 是没有返回值的，所以只要执行就行)
        try {
            PrologQuery bdgs = pl_.query(query);
            for (auto const& solution : bdgs) { break; } // 触发执行
            res.confirmation = true;
        } catch (exception &e) {
            ROS_ERROR_STREAM("Prolog assertion failed: " << e.what());
            res.confirmation = false;
        }
        
        if(m_query_flag_save) saveQueryToFile(query);
        return true;
    }

    // ---------------------------------------------------------
    // Callback: 查知识 (Learning -> Reasoning)
    // ---------------------------------------------------------
    bool srv_query_callback(world_percept_assig4::QueryKnowledge::Request &req,
                            world_percept_assig4::QueryKnowledge::Response &res)
    {
        std::string query;

        if(req.target_class.find("CHECK_FOUND:") == 0){
            std::string real_target = req.target_class.substr(12);
            ROS_INFO_STREAM("Reasoning: checking if [" << real_target << "] is found...");

            query = "target_found('" + real_target + "', Obj)";
            PrologQuery bdgs = pl_.query(query);

            for (auto const& solution : bdgs)
            {
                std::string obj_name = solution["Obj"].as<std::string>();
                res.likely_locations.push_back(obj_name);
            }
            return true;

        }

        ROS_INFO_STREAM("Reasoning: Querying candidates for [" << req.target_class << "]");

        // 构造 Prolog 查询
        // 使用你的 fp_reasoning.pl 中的谓词: candidate_place(Target, Place).
        // 这会返回所有符合逻辑的候选地点（例如：找盘子 -> 返回所有桌子）
        std::string queryPlace = "candidate_place('" + req.target_class + "', Place), \\+ visited(Place)";
        
        // 如果你想直接用 decide_search_order 也可以，但那个返回的是列表，处理起来稍复杂
        // 这里我们先获取所有候选点，交给 Learning Node 去排序 (FP.T02)

        PrologQuery bdgs = pl_.query(queryPlace);

        for (auto const& solution : bdgs)
        {
            // 提取 Place 变量
            std::string loc = solution["Place"].as<std::string>();
            res.likely_locations.push_back(loc);
        }

        if (res.likely_locations.empty()) {
            ROS_WARN("Reasoning: No logical candidates found (Maybe need to explore first?).");
        } else {
            ROS_INFO_STREAM("Reasoning: Found " << res.likely_locations.size() << " candidates.");
        }

        if(m_query_flag_save) saveQueryToFile(queryPlace);
        return true;
    }

    void saveQueryToFile(std::string query)
    {
        if (m_query_flag_save && !m_query_file_name.empty())
        {
            std::ofstream outfile(m_query_file_name.c_str(), std::ios::app);
            if (outfile.is_open()) outfile << query << "." << std::endl;
        }
    }

}; 

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "reasoning_node");
  ros::NodeHandle nh;   
  Reasoner myReasoner(nh);

  // 处理参数 (保留)
  if (argc >= 2) {
      std::string saveFilePath = argv[1]; 
      bool saveQueries_flag; 
      if (!nh.getParam("read_prolog_queries/save_flag", saveQueries_flag)) saveQueries_flag = false;

      if(saveQueries_flag) {
           std::string savedQueryFile;
           if (nh.getParam("read_prolog_queries/saved_query_file", savedQueryFile)) {
               std::string fullPath = saveFilePath + savedQueryFile;
               myReasoner.setOutQueriesFile(fullPath);
           }
      }
  }
  ros::spin();
  return 0;
}