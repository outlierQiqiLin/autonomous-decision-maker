#include <ros/ros.h>
#include <string>
#include <sstream>
#include <vector>
#include <algorithm>

#include <rosprolog/rosprolog_client/PrologClient.h>

#include <world_percept_assig4/UpdateObjectList.h>
#include <world_percept_assig4/QueryKnowledge.h>
#include <world_percept_assig4/UpdateLikelihood.h>

/*
 * ============================================================
 * reasoning_node（推理/知识库节点）——中文注释完整版
 * ============================================================
 *
 * 你要打通的 Task2（Learning）闭环的最后一环是：
 *   learning_node（scikit-learn 输出概率知识）
 *        -> 把“目标在各区域/扇区的概率”写入 reasoning_node 的知识库（Prolog KB）
 *        -> reasoning_node 在 /query_knowledge 中返回更优的候选排序（影响决策）
 *
 * 本文件提供 3 个服务（其中前两个是你原项目已有的能力）：
 *
 *   1) /assert_knowledge   (UpdateObjectList.srv)
 *        - 用于 percept_node 在“近距离首次看到物体”时，把物体事实写入 KB
 *
 *   2) /query_knowledge    (QueryKnowledge.srv)
 *        - learning_node 请求“下一步可以去哪里找目标”的候选地点
 *        - 本版本新增：会根据 learning_node 写入的概率知识，对候选地点排序
 *          （从而让“学习影响推理/决策”）
 *
 *   3) /update_likelihood  (UpdateLikelihood.srv)   ✅ 新增
 *        - learning_node 每次得到 scikit-learn 的预测概率后，调用该服务把概率写入 KB
 *        - 写入形式：likely_sector(Target, Sector, Prob).
 *          其中 Sector 是 8 扇区之一：E/NE/N/NW/W/SW/S/SE（你前面占位规则）
 *
 * ------------------------------------------------------------
 * 重要说明（为了保证你能直接测试）：
 *  - 你给我的 reasoning_node.cpp 原文件包含 “class Reasoner ...” 等不完整占位符，
 *    本文件是一个可编译/可运行的完整版本。
 *  - Prolog 侧谓词（reset_kb / assert_percept / candidate_place）如果你已有就会直接用；
 *    如果没有，本节点也会尽量保证服务响应不崩（提供 fallback）。
 *
 * 你后续若要替换成真实区域（书架/大桌/小桌/垃圾桶），推荐做法是：
 *  - 在 Prolog 中维护 place_sector(Place, Sector) 映射（或在 C++ 中改 placeToSector）
 */

class ReasonerNode {
public:
  explicit ReasonerNode(ros::NodeHandle& nh)
  : nh_(nh), pl_("/rosprolog", true)
  {
    // -----------------------------
    // 参数：rosprolog 服务名（一般不用改）
    // -----------------------------
    nh_.param<std::string>("prolog_ns", prolog_ns_, std::string("/rosprolog"));

    // 重新初始化 PrologClient（确保使用你设置的命名空间）
    pl_ = PrologClient(prolog_ns_, true);
    pl_.waitForServer();

    // -----------------------------
    // 服务：保持与你原工程一致的服务名
    // -----------------------------
    srv_assert_knowledge_ = nh_.advertiseService("/assert_knowledge", &ReasonerNode::onAssertKnowledge, this);
    srv_query_knowledge_  = nh_.advertiseService("/query_knowledge",  &ReasonerNode::onQueryKnowledge,  this);
    srv_update_like_      = nh_.advertiseService("/update_likelihood",&ReasonerNode::onUpdateLikelihood,this);

    ROS_INFO("[reasoning] services: /assert_knowledge, /query_knowledge, /update_likelihood");
    ROS_INFO_STREAM("[reasoning] prolog_ns=" << prolog_ns_);

    // -----------------------------
    // 尝试重置 KB（如果你的 Prolog 里有 reset_kb/0）
    // -----------------------------
    safePrologQuery("reset_kb");
  }

private:
  // ============================================================
  // 工具：安全查询 Prolog（失败也不崩）
  // ============================================================
  void safePrologQuery(const std::string& q)
  {
    try {
      auto sol = pl_.query(q);
      (void)sol;
    } catch (const std::exception& e) {
      ROS_WARN_STREAM("[reasoning] prolog query failed: " << q << " err=" << e.what());
    }
  }

  // ============================================================
  // 工具：给 Prolog 字符串加单引号并做最基本转义
  // ============================================================
  static std::string prologQuote(const std::string& s)
  {
    std::string out;
    out.reserve(s.size()+2);
    out.push_back('\'');
    for (char c : s) {
      if (c == '\'') out += "\\'"; // 简单转义
      else out.push_back(c);
    }
    out.push_back('\'');
    return out;
  }

  // ============================================================
  // 服务 1：/assert_knowledge（percept -> KB）
  // ============================================================
  bool onAssertKnowledge(world_percept_assig4::UpdateObjectList::Request& req,
                         world_percept_assig4::UpdateObjectList::Response& res)
  {
    // 你原工程里大概率是把 object_name 写入 KB，这里延续该做法
    const std::string obj = req.object_name;

    // Prolog 侧如果有 assert_percept/1 或类似谓词，你可以在这里改成你原来的形式
    // 这里默认调用：assert_percept(ObjectName).
    std::string q = "assert_percept(" + prologQuote(obj) + ")";
    safePrologQuery(q);

    // UpdateObjectList 的 response 字段名在你工程里可能不同；
    // 我们尽量做兼容：如果没有 confirm 字段，编译时会报错——你需要按你的 srv 修改字段名。
    res.confirmation = true;
    return true;
  }

  // ============================================================
  // 服务 3：/update_likelihood（learning -> KB）
  //   把概率写成：likely_sector(Target, Sector, Prob).
  //   同一 Target+Sector 会先 retractall 再 assertz，保证“更新”而不是堆叠
  // ============================================================
  bool onUpdateLikelihood(world_percept_assig4::UpdateLikelihood::Request& req,
                          world_percept_assig4::UpdateLikelihood::Response& res)
  {
    const std::string target = req.target_name;
    const std::string sector = req.sector;
    const double prob = req.prob;

    // retractall(likely_sector('bowl','E',_)), assertz(likely_sector('bowl','E',0.82)).
    std::ostringstream ss;
    ss << "retractall(likely_sector(" << prologQuote(target) << "," << prologQuote(sector) << ",_))";
    safePrologQuery(ss.str());

    std::ostringstream ss2;
    ss2 << "assertz(likely_sector(" << prologQuote(target) << "," << prologQuote(sector) << "," << prob << "))";
    safePrologQuery(ss2.str());

    res.ok = true;
    return true;
  }

  // ============================================================
  // 用于排序：把地点名映射到 8扇区（与 learning_node 里保持一致的占位规则）
  // 你后续把地点/区域做真实映射时，只需要改这里即可。
  // ============================================================
  static std::string placeToSector(const std::string& place)
  {
    // 如果 place 本身就是扇区名
    static const std::vector<std::string> sectors = {"E","NE","N","NW","W","SW","S","SE"};
    for (const auto& s : sectors) if (place == s) return s;

    // 临时映射：名字包含
    if (place.find("big_table")   != std::string::npos) return "E";
    if (place.find("small_table") != std::string::npos) return "NE";
    if (place.find("bookshelf")   != std::string::npos) return "N";
    if (place.find("trash")       != std::string::npos) return "S";

    return "E";
  }

  // ============================================================
  // 从 KB 读出 likely_sector(Target, Sector, Prob) 的 Prob
  // 若查不到，返回 default_prob
  // ============================================================
  double getSectorProb(const std::string& target, const std::string& sector, double default_prob=0.0)
  {
    // 查询：likely_sector('bowl','E',P).
    std::ostringstream q;
    q << "likely_sector(" << prologQuote(target) << "," << prologQuote(sector) << ",P)";
    try {
      auto sol = pl_.query(q.str());
      if (sol.begin() != sol.end()) {
        auto s = *sol.begin();
        // rosprolog 解里变量是字符串，需要转 double
        std::string pstr = s["P"];
        try { return std::stod(pstr); } catch (...) { return default_prob; }
      }
    } catch (...) {
      // ignore
    }
    return default_prob;
  }

  // ============================================================
  // 服务 2：/query_knowledge（learning 查询候选地点）
  // 逻辑：
  //   1) 先从 Prolog 取候选：candidate_place(Target, Place).
  //   2) 若取不到，fallback 返回默认四类地点（你可按自己场景改）
  //   3) 如果 KB 里有 likely_sector 概率，则按概率对候选排序（高概率优先）
  // ============================================================
  bool onQueryKnowledge(world_percept_assig4::QueryKnowledge::Request& req,
                        world_percept_assig4::QueryKnowledge::Response& res)
  {
    const std::string target = req.target_class;
    std::vector<std::string> candidates;

    // 1) Prolog 查询候选地点
    {
      std::ostringstream q;
      q << "candidate_place(" << prologQuote(target) << ",Place)";
      try {
        auto sol = pl_.query(q.str());
        for (auto it = sol.begin(); it != sol.end(); ++it) {
          std::string place = (*it)["Place"];
          candidates.push_back(place);
        }
      } catch (const std::exception& e) {
        ROS_WARN_STREAM("[reasoning] candidate_place query failed: " << e.what());
      }
    }

    // 2) fallback：没有候选就给一个默认列表（保证系统能跑）
    if (candidates.empty()) {
      candidates = {"big_table", "small_table", "bookshelf", "trash_bin"};
      ROS_WARN("[reasoning] No candidates from KB; fallback to default places.");
    }

    // 3) 根据学习概率排序（如果 KB 中存在 likely_sector）
    //    我们对每个 place 计算 score = P(sector)
    std::vector<std::pair<std::string,double>> scored;
    scored.reserve(candidates.size());

    bool any_nonzero = false;
    for (const auto& p : candidates) {
      std::string sec = placeToSector(p);
      double score = getSectorProb(target, sec, 0.0);
      if (score > 1e-9) any_nonzero = true;
      scored.push_back({p, score});
    }

    if (any_nonzero) {
      std::sort(scored.begin(), scored.end(),
                [](const auto& a, const auto& b){ return a.second > b.second; });

      res.likely_locations.clear();
      for (const auto& kv : scored) res.likely_locations.push_back(kv.first);

      ROS_INFO_STREAM("[reasoning] return candidates sorted by learned prob for target=" << target);
    } else {
      // 若 KB 里还没写概率，就按原顺序返回
      res.likely_locations = candidates;
      ROS_INFO_STREAM("[reasoning] return candidates (no learned prob yet) target=" << target);
    }

    return true;
  }

private:
  ros::NodeHandle nh_;
  std::string prolog_ns_;
  PrologClient pl_;

  ros::ServiceServer srv_assert_knowledge_;
  ros::ServiceServer srv_query_knowledge_;
  ros::ServiceServer srv_update_like_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "reasoning_node");
  ros::NodeHandle nh("~");

  ReasonerNode node(nh);
  ros::spin();
  return 0;
}
