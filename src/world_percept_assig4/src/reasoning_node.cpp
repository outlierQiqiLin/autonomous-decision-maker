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
    // 修改版：支持万能查询，方便测试 Task 1 所有谓词
    // ---------------------------------------------------------
    bool srv_query_callback(world_percept_assig4::QueryKnowledge::Request &req,
                            world_percept_assig4::QueryKnowledge::Response &res)
    {
        // [关键修改] 直接把 req.target_class 当作完整的 Prolog 查询语句
        // 例如：req.target_class 可能是 "support_surface(X)."
        std::string query = req.target_class;
        
        ROS_INFO_STREAM("Reasoning: Executing Raw Query [" << query << "]");

        try {
            PrologQuery bdgs = pl_.query(query);

            for (auto const& solution : bdgs)
            {
                // 我们假设查询结果里都有一个变量叫 "Res" 或者 "X" 或者 "Place"
                // 为了通用，我们遍历所有返回的变量，把它拼成字符串
                std::string result_str = "";
                
                for (auto const& val : solution)
                {
                    // val.first 是变量名 (如 X), val.second 是值 (如 table_1)
                    if (!result_str.empty()) result_str += ", ";
                    result_str += val.second.as<std::string>();
                }
                
                if (!result_str.empty())
                {
                    res.likely_locations.push_back(result_str);
                    ROS_INFO_STREAM(" -> Result: " << result_str);
                }
            }
        } 
        catch (exception &e) 
        {
            ROS_ERROR_STREAM("Query failed: " << e.what());
        }

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