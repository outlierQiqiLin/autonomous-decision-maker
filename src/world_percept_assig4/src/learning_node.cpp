#include <ros/ros.h>
#include <std_msgs/String.h>

#include <world_percept_assig4/GotoObject.h>
#include <world_percept_assig4/QueryKnowledge.h>
#include <world_percept_assig4/TrainPredict.h>   // ✅ 新增：Python(scikit-learn) 服务接口

#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <ctime>
#include <algorithm>
#include <sys/stat.h>

/*
 * ============================================================
 * learning_node（学习节点）——直接使用 scikit-learn（DecisionTreeClassifier）版本
 * ============================================================
 *
 * 你要求的闭环（可直接测试）：
 *   percept_node   发布 /percept/observations（std_msgs/String）
 *                 格式：object_name,region_id,range_m
 *
 *   learning_node  订阅观测 -> 维护本次 episode 的探索特征 visited_* -> 找到目标后写 CSV（数据集）
 *                 每次做决策时，调用 Python 节点 /ml/train_predict：
 *                   - Python 节点读取 CSV
 *                   - 训练/评估 DecisionTreeClassifier（scikit-learn）
 *                   - 输出 8 个扇区（E/NE/N/...）的预测概率
 *                 learning_node 用概率对候选地点排序，选最优动作，并调用 GotoObject 执行
 *
 *   reasoning_node 提供 /query_knowledge 返回候选地点（保留原功能）
 *
 * ------------------------------------------------------------
 * 重要说明：
 * 1) 由于 scikit-learn 是 Python 库，本 C++ 节点不会直接链接 scikit-learn，
 *    而是通过 ROS Service 调用 Python 节点（ml_node.py）。
 *
 * 2) 本版本不再使用“概率文件占位”，而是“每次决策都直接调用 Python 服务”。
 *
 * 3) 数据集文件 CSV 作为“学习证据”保存到本地（默认 ~/.ros/bowl_dataset.csv）。
 *    你可以在报告里展示：
 *       - CSV 数据增长
 *       - Python 输出的 confusion matrix / accuracy（在 rosconsole 里可见）
 *       - 学习后策略更快更偏向高概率区域
 *
 * ------------------------------------------------------------
 * 你之后要替换成真实区域（书架/大桌/小桌/垃圾桶）时：
 *   - 现在我们用 8 扇区 region_id（E/NE/N/NW/W/SW/S/SE）
 *   - reasoning 返回的地点名（例如 big_table）需要映射到扇区
 *     -> placeToSector(...) 目前是占位映射（字符串包含），后续你按实际改
 */

class LearningNode
{
public:
  explicit LearningNode(ros::NodeHandle& nh)
  : nh_(nh)
  {
    // ========== 1) 参数 ==========
    nh_.param<std::string>("srv_query_knowledge_name", srv_query_knowledge_name_, std::string("/query_knowledge"));
    nh_.param<std::string>("srv_goto_object_name",    srv_goto_object_name_,    std::string("GotoObject"));

    nh_.param<std::string>("srv_ml_name",             srv_ml_name_,             std::string("/ml/train_predict"));
    nh_.param<std::string>("sub_obs_topic_name",      sub_obs_topic_name_,      std::string("/percept/observations"));

    // 学习数据集路径（本地 CSV）
    nh_.param<std::string>("csv_path", csv_path_, std::string("~/.ros/bowl_dataset.csv"));

    // 目标物体名（默认 bowl）
    nh_.param<std::string>("default_target", default_target_, std::string("bowl"));

    // ========== 2) ROS 通信对象 ==========
    client_reasoning_ = nh_.serviceClient<world_percept_assig4::QueryKnowledge>(srv_query_knowledge_name_);
    client_control_   = nh_.serviceClient<world_percept_assig4::GotoObject>(srv_goto_object_name_);
    client_ml_        = nh_.serviceClient<world_percept_assig4::TrainPredict>(srv_ml_name_);

    sub_obs_ = nh_.subscribe(sub_obs_topic_name_, 200, &LearningNode::obsCb, this);

    ROS_INFO("Learning Node (scikit-learn) initialized. Waiting for services...");
    ros::service::waitForService(srv_query_knowledge_name_);
    ros::service::waitForService(srv_goto_object_name_);
    ros::service::waitForService(srv_ml_name_);
    ROS_INFO("Services connected.");

    std::srand((unsigned int)std::time(nullptr));
    resetEpisodeState();
  }

  void run_mission(const std::string& target_object)
  {
    current_target_ = target_object;
    ROS_INFO_STREAM("=== MISSION START: Find [" << current_target_ << "] ===");

    resetEpisodeState();

    // 2 秒一次决策（你可按仿真速度调整）
    ros::Rate rate(0.5);

    while (ros::ok())
    {
      ros::spinOnce();

      if (found_target_) {
        ROS_INFO_STREAM("Target [" << current_target_ << "] found in sector [" << found_sector_ << "]. Mission complete.");
        break;
      }

      // 1) 向 reasoning 获取候选地点（保留原功能）
      std::vector<std::string> candidates = queryCandidatesFromReasoning();
      if (candidates.empty()) {
        // 没候选就随机探索一个扇区（占位）
        std::string fallback = randomFallbackSector();
        ROS_WARN_STREAM("No candidates from reasoning. Random explore sector: " << fallback);
        // 你如果 control 只能去“具体地点”，这里就需要把扇区映射到一个导航点/物体
        // 当前先保持行为不崩（仍调用 GotoObject）
        callGotoObject(fallback);
      } else {
        // 2) 调用 Python(scikit-learn) 服务，得到 8 扇区概率（学习结果）
        auto probs = callMLTrainPredict();

        // 3) 用学习结果选最优地点
        std::string best = selectBestLocation(candidates, probs);
        ROS_INFO_STREAM("Learning-selected best target: " << best);
        callGotoObject(best);
      }

      ros::spinOnce();
      rate.sleep();
    }
  }

private:
  // ============================================================
  // 观测订阅：更新 visited_*；若观测到目标，写 CSV
  // ============================================================
  void obsCb(const std_msgs::String::ConstPtr& msg)
  {
    std::string obj, region;
    double range_m = 0.0;

    if (!parseObservation(msg->data, obj, region, range_m)) return;

    // 更新 visited
    if (visited_.count(region) > 0) visited_[region] = 1;

    // 发现目标 -> 记录 episode -> 写 CSV
    if (!current_target_.empty() && obj == current_target_) {
      if (!found_target_) {
        found_target_ = true;
        found_sector_ = region;

        ROS_INFO_STREAM("[learning] Observed target [" << current_target_
                        << "] in sector [" << found_sector_
                        << "] (range=" << range_m << "m). Logging episode to CSV...");

        appendEpisodeToCSV(found_sector_);
      }
    }
  }

  bool parseObservation(const std::string& s, std::string& obj, std::string& region, double& range_m)
  {
    std::stringstream ss(s);
    std::string token;
    std::vector<std::string> parts;
    while (std::getline(ss, token, ',')) parts.push_back(token);
    if (parts.size() < 2) return false;

    obj = parts[0];
    region = parts[1];

    if (parts.size() >= 3) {
      try { range_m = std::stod(parts[2]); }
      catch (...) { range_m = 0.0; }
    } else range_m = 0.0;

    return true;
  }

  // ============================================================
  // 访问 reasoning：获取候选地点
  // ============================================================
  std::vector<std::string> queryCandidatesFromReasoning()
  {
    std::vector<std::string> out;

    world_percept_assig4::QueryKnowledge srv;
    srv.request.target_class = current_target_;

    if (client_reasoning_.call(srv)) {
      out = srv.response.likely_locations;
      ROS_INFO_STREAM("[learning] Candidates from reasoning: " << out.size());
    } else {
      ROS_WARN("[learning] Failed to call /query_knowledge.");
    }
    return out;
  }

  // ============================================================
  // 调用 Python(scikit-learn) 服务：训练/预测 -> 返回 8扇区概率
  // ============================================================
  std::unordered_map<std::string, double> callMLTrainPredict()
  {
    world_percept_assig4::TrainPredict srv;
    srv.request.target_name = current_target_;

    // 将 visited_* 作为特征输入
    srv.request.visited_E  = visited_["E"];
    srv.request.visited_NE = visited_["NE"];
    srv.request.visited_N  = visited_["N"];
    srv.request.visited_NW = visited_["NW"];
    srv.request.visited_W  = visited_["W"];
    srv.request.visited_SW = visited_["SW"];
    srv.request.visited_S  = visited_["S"];
    srv.request.visited_SE = visited_["SE"];

    std::unordered_map<std::string, double> probs;

    if (client_ml_.call(srv)) {
      probs["E"]  = srv.response.p_E;
      probs["NE"] = srv.response.p_NE;
      probs["N"]  = srv.response.p_N;
      probs["NW"] = srv.response.p_NW;
      probs["W"]  = srv.response.p_W;
      probs["SW"] = srv.response.p_SW;
      probs["S"]  = srv.response.p_S;
      probs["SE"] = srv.response.p_SE;

      // 打印（方便你在视频/日志里展示“学习输出的知识”）
      ROS_INFO_STREAM("[learning] ML probs: "
        << "E="<<probs["E"]<<" "
        << "NE="<<probs["NE"]<<" "
        << "N="<<probs["N"]<<" "
        << "NW="<<probs["NW"]<<" "
        << "W="<<probs["W"]<<" "
        << "SW="<<probs["SW"]<<" "
        << "S="<<probs["S"]<<" "
        << "SE="<<probs["SE"]);
    } else {
      ROS_WARN("[learning] Failed to call /ml/train_predict. Fallback to uniform probs.");
      // fallback：均匀分布，保证系统不崩
      const double u = 1.0/8.0;
      for (auto &kv : visited_) probs[kv.first] = u;
    }

    return probs;
  }

  // ============================================================
  // 在 candidates 中选择最优地点（学习影响决策的核心）
  // ============================================================
  std::string selectBestLocation(const std::vector<std::string>& candidates,
                                 const std::unordered_map<std::string, double>& probs)
  {
    double best_score = -1.0;
    std::string best_loc;

    for (const auto& loc : candidates) {
      const std::string sector = placeToSector(loc);
      double score = 0.0;

      auto it = probs.find(sector);
      if (it != probs.end()) score = it->second;

      ROS_INFO_STREAM("  Candidate: " << loc << " (mapped_sector=" << sector << ") score=" << score);

      if (score > best_score) {
        best_score = score;
        best_loc = loc;
      }
    }

    // 如果全部 score 都为 0（极端情况），就随机选一个，保证继续跑
    if (best_loc.empty() && !candidates.empty()) best_loc = candidates[std::rand()%candidates.size()];
    return best_loc;
  }

  // ============================================================
  // 将 reasoning 返回的地点名映射到 8扇区（占位逻辑，后续你按实际改）
  // ============================================================
  std::string placeToSector(const std::string& loc) const
  {
    // 如果 loc 本身就是扇区名，直接返回（例如你将 reasoning 改成直接返回 E/NE/...）
    if (visited_.count(loc) > 0) return loc;

    // 否则做一个临时映射（基于名字包含）
    if (loc.find("big_table")   != std::string::npos) return "E";
    if (loc.find("small_table") != std::string::npos) return "NE";
    if (loc.find("bookshelf")   != std::string::npos) return "N";
    if (loc.find("trash")       != std::string::npos) return "S";

    // 默认 E
    return "E";
  }

  // ============================================================
  // 写 CSV：每次找到目标后写入一行样本（visited_* + y_sector）
  // ============================================================
  static bool fileExists(const std::string& path)
  {
    struct stat buffer;
    return (stat(path.c_str(), &buffer) == 0);
  }

  static std::string expandUserPath(const std::string& path)
  {
    if (!path.empty() && path[0] == '~') {
      const char* home = std::getenv("HOME");
      if (home) return std::string(home) + path.substr(1);
    }
    return path;
  }

  void appendEpisodeToCSV(const std::string& y_sector)
  {
    const std::string csv_path = expandUserPath(csv_path_);
    const bool exists = fileExists(csv_path);

    std::ofstream ofs(csv_path, std::ios::app);
    if (!ofs.is_open()) {
      ROS_WARN_STREAM("[learning] Cannot open CSV for append: " << csv_path);
      return;
    }

    if (!exists) {
      ofs << "visited_E,visited_NE,visited_N,visited_NW,visited_W,visited_SW,visited_S,visited_SE,y_sector\n";
    }

    ofs << visited_["E"]  << "," << visited_["NE"] << "," << visited_["N"]  << "," << visited_["NW"] << ","
        << visited_["W"]  << "," << visited_["SW"] << "," << visited_["S"]  << "," << visited_["SE"] << ","
        << y_sector << "\n";

    ofs.close();

    ROS_INFO_STREAM("[learning] Appended episode to CSV: " << csv_path << " label=" << y_sector);
  }

  // ============================================================
  // 调 control：GotoObject
  // ============================================================
  void callGotoObject(const std::string& obj_name)
  {
    world_percept_assig4::GotoObject srv;
    srv.request.object_name = obj_name;
    srv.request.start = 1;

    if (client_control_.call(srv)) {
      if (srv.response.confirm) {
        ROS_INFO_STREAM("[learning] Robot moving towards: " << obj_name);
        // 原项目里用 sleep 模拟移动耗时：保留
        ros::Duration(15.0).sleep();
      } else {
        ROS_WARN("[learning] Control rejected (confirm=false).");
      }
    } else {
      ROS_WARN("[learning] Failed to call GotoObject.");
    }
  }

  std::string randomFallbackSector() const
  {
    static const std::vector<std::string> sectors = {"E","NE","N","NW","W","SW","S","SE"};
    return sectors[std::rand() % sectors.size()];
  }

  void resetEpisodeState()
  {
    visited_.clear();
    visited_["E"]=0; visited_["NE"]=0; visited_["N"]=0; visited_["NW"]=0;
    visited_["W"]=0; visited_["SW"]=0; visited_["S"]=0; visited_["SE"]=0;

    found_target_ = false;
    found_sector_.clear();
  }

private:
  ros::NodeHandle nh_;

  ros::ServiceClient client_reasoning_;
  ros::ServiceClient client_control_;
  ros::ServiceClient client_ml_;
  ros::Subscriber sub_obs_;

  // 参数
  std::string srv_query_knowledge_name_;
  std::string srv_goto_object_name_;
  std::string srv_ml_name_;
  std::string sub_obs_topic_name_;
  std::string csv_path_;
  std::string default_target_;

  // 当前任务目标
  std::string current_target_;

  // episode 状态
  std::unordered_map<std::string,int> visited_;
  bool found_target_ = false;
  std::string found_sector_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "learning_node");
  ros::NodeHandle nh("~");

  LearningNode node(nh);

  ros::Duration(2.0).sleep();

  std::string target;
  nh.param<std::string>("default_target", target, std::string("bowl"));
  node.run_mission(target);

  ros::spin();
  return 0;
}
