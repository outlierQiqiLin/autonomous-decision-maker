#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <std_msgs/String.h>

#include <world_percept_assig4/UpdateObjectList.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <cmath>
#include <string>
#include <unordered_set>
#include <sstream>
#include <algorithm>

/*
 * ============================================================
 * percept_node（感知节点）——中文注释完整版
 * ============================================================
 *
 * 你当前的项目目标（Task2：Learning）需要形成如下的数据链路：
 *   percept_node（感知） -> learning_node（学习/数据收集/模型迭代） -> reasoning_node（知识库/推理/决策）
 *
 * 本文件做的事情：
 *   A) 订阅 /gazebo/model_states，获取机器人与场景中所有模型（物体）的位姿
 *   B) 当机器人“近距离第一次看到”某个物体时：
 *        1) 仍然沿用你原有的方式：调用服务 /assert_knowledge（UpdateObjectList.srv）
 *           把物体名字写入 reasoning_node 的 Prolog 知识库
 *        2) 额外发布一个话题 /percept/observations（std_msgs/String）
 *           让 learning_node 能订阅到“观测数据”，用于写 CSV / 训练 scikit-learn / 迭代学习
 *
 * 【为什么需要发布 /percept/observations？】
 *   - 如果 percept_node 只写 Prolog，而 learning_node 没有收到“原始观测流”，那么 learning_node 很难：
 *       - 记录 episode 过程
 *       - 构造训练数据（CSV）
 *       - 做学习模型迭代并输出概率知识
 *
 * 【区域划分（占位方案）说明：8方向 + 10m】
 *   - 你暂时不知道真实的“书架/大桌/小桌/垃圾桶”的空间范围，这很正常。
 *   - 这里先使用一个占位规则，保证整个学习闭环能先跑通：
 *       以机器人【初始位姿】为参考，在半径 10m 内，把空间按角度分成 8 个扇区：
 *         E, NE, N, NW, W, SW, S, SE
 *     其中：
 *       - E：初始朝向的正前方（相当于 x+ 方向）
 *       - N：初始朝向的左侧（相当于 y+ 方向）
 *       - S：初始朝向的右侧（相当于 y- 方向）
 *   - 后续你只需要替换 classifyRegion(...) 这部分即可，把扇区规则换成真实区域（书架/桌子等）。
 *
 * 【/percept/observations 发布格式】
 *   - 由于先追求“最少改动、快速跑通”，这里用 std_msgs/String，不引入自定义 msg。
 *   - 字符串格式为：
 *       object_name,region_id,range_m
 *     例：
 *       bowl,NE,4.23
 *
 * 提醒：
 *   - 本节点需要知道“机器人模型名”来从 model_states 里提取机器人 pose。
 *   - 默认机器人模型名为 "tiago"，如不一致，请通过参数 ~robot_model_name 修改。
 */

class PerceptNode {
public:
  explicit PerceptNode(ros::NodeHandle& nh)
  : nh_(nh)
  {
    // -----------------------------
    // 1) 读取参数（可在 启动文件/参数服务器 修改）
    // -----------------------------
    nh_.param<std::string>("subs_topic_name", subs_topic_name_, std::string("/gazebo/model_states"));
    nh_.param<std::string>("pub_obs_topic_name", pub_obs_topic_name_, std::string("/percept/observations"));
    nh_.param<std::string>("srv_assert_knowledge_name", srv_assert_knowledge_name_, std::string("/assert_knowledge"));
    nh_.param<std::string>("robot_model_name", robot_model_name_, std::string("tiago"));

    // vision_range_m_：当物体到机器人距离小于这个阈值，我们认为“机器人看到了该物体”
    // 用途：避免太远的物体也被当作“已经看到”，导致写入知识库过早/过多
    nh_.param<double>("vision_range_m", vision_range_m_, 1.1);

    // sector_max_range_m_：扇区划分的最大半径
    // 10 米 外的物体可以标为 FAR（你也可以选择直接忽略不发布）
    nh_.param<double>("sector_max_range_m", sector_max_range_m_, 10.0);

    // -----------------------------
    // 2) 初始化 发布者 / 订阅者 / 服务客户端
    // -----------------------------
    pub_obs_ = nh_.advertise<std_msgs::String>(pub_obs_topic_name_, 50);

    sub_model_states_ = nh_.subscribe(subs_topic_name_, 10, &PerceptNode::modelStatesCb, this);

    client_assert_kb_ = nh_.serviceClient<world_percept_assig4::UpdateObjectList>(srv_assert_knowledge_name_);

    ROS_INFO_STREAM("[percept] subscribe=" << subs_topic_name_
                    << " publish=" << pub_obs_topic_name_
                    << " srv=" << srv_assert_knowledge_name_
                    << " robot_model_name=" << robot_model_name_);
  }

private:
  // -----------------------------
  // 工具函数：从四元数提取 航向角(yaw)（航向角）
  // -----------------------------
  static double yawFromQuat(const geometry_msgs::Quaternion& qmsg) {
    tf2::Quaternion q(qmsg.x, qmsg.y, qmsg.z, qmsg.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return yaw;
  }

  // -----------------------------
  // 工具函数：把 (dx,dy) 旋转到“初始朝向坐标系”
  //   - 初始朝向的正前方当作 x 正方向
  //   - 初始朝向左侧当作 y 正方向
  // 这样 “角度=0” 就表示初始朝向正前方（E）
  // -----------------------------
  static void rotateIntoStartFrame(double dx, double dy, double yaw0,
                                   double &dx_r, double &dy_r) {
    // 等价于旋转 -航向角(yaw)0
    const double c = std::cos(-yaw0);
    const double s = std::sin(-yaw0);
    dx_r = c*dx - s*dy;
    dy_r = s*dx + c*dy;
  }

  // -----------------------------
  // 工具函数：把角度映射到 8 个扇区
  // 输入：
  //   ang_rad：[-pi, pi]，0 表示初始朝向正前方
  // 输出：
  //   "E","NE","N","NW","W","SW","S","SE"
  // -----------------------------
  static std::string sector8FromAngle(double ang_rad) {
    const double pi = M_PI;

    // 为了“就近取整”到 8 个扇区，先平移 22.5°
    double a = ang_rad + pi/8.0;

    // 把角度归一化到 [-pi, pi]
    while (a <= -pi) a += 2*pi;
    while (a >   pi) a -= 2*pi;

    // bin 宽度为 45°（pi/4）
    int idx = (int)std::floor((a + pi) / (pi/4.0));
    idx = std::max(0, std::min(7, idx));

    // 这个顺序保证：
    //   a=-pi   -> W
    //   a=0     -> E
    //   a=+pi/2 -> N
    //   a=-pi/2 -> S
    static const char* names[8] = {"W","SW","S","SE","E","NE","N","NW"};
    return std::string(names[idx]);
  }

  // -----------------------------
  // 区域分类函数（占位版本：8扇区 + 10 米）
  // 你未来要换成真实的“书架/桌子/垃圾桶范围”时，就改这个函数即可。
  // -----------------------------
  std::string classifyRegion(double obj_x, double obj_y) const {
    if (!have_start_pose_) return "UNKNOWN";

    const double dx = obj_x - x0_;
    const double dy = obj_y - y0_;
    const double r  = std::sqrt(dx*dx + dy*dy);

    if (r > sector_max_range_m_) {
      return "FAR";  // 超过 10 米：先标记 FAR（也可以选择不发布）
    }

    double dx_r, dy_r;
    rotateIntoStartFrame(dx, dy, yaw0_, dx_r, dy_r);
    const double ang = std::atan2(dy_r, dx_r);
    return sector8FromAngle(ang);
  }

  // -----------------------------
  // 回调：处理 /gazebo/model_states
  // -----------------------------
  void modelStatesCb(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    // 1) 从 msg->name[] 中找到机器人对应的索引
    int robot_idx = -1;
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] == robot_model_name_) {
        robot_idx = (int)i;
        break;
      }
    }

    // 找不到机器人模型名，则无法计算距离与扇区
    if (robot_idx < 0) {
      ROS_WARN_THROTTLE(2.0,
        "[percept] 在 /gazebo/model_states 中找不到机器人模型名 '%s'。请设置参数 ~robot_model_name。",
        robot_model_name_.c_str());
      return;
    }

    const auto& robot_pose = msg->pose[robot_idx];

    // 2) 只在第一次回调时记录“初始位姿”
    if (!have_start_pose_) {
      x0_ = robot_pose.position.x;
      y0_ = robot_pose.position.y;
      yaw0_ = yawFromQuat(robot_pose.orientation);
      have_start_pose_ = true;
      ROS_INFO_STREAM("[percept] 已记录初始位姿 x0=" << x0_ << " y0=" << y0_ << " yaw0=" << yaw0_);
    }

    // 3) 遍历所有模型（物体）
    for (size_t i = 0; i < msg->name.size(); ++i) {
      const std::string& obj_name = msg->name[i];

      // 忽略机器人自身
      if (obj_name == robot_model_name_) continue;

      // 你可以在这里加入更多忽略项（比如地面、墙体等）
      if (obj_name.find("ground") != std::string::npos) continue;
      if (obj_name.find("wall")   != std::string::npos) continue;

      const auto& obj_pose = msg->pose[i];

      // 计算物体到机器人的距离（用于“是否在视觉范围内”）
      const double dx_rbt = obj_pose.position.x - robot_pose.position.x;
      const double dy_rbt = obj_pose.position.y - robot_pose.position.y;
      const double dist_to_robot = std::sqrt(dx_rbt*dx_rbt + dy_rbt*dy_rbt);

      // -----------------------------
      // 3.1 发布给 learning_node 的观测（不要求近距离，只要在 10 米 内即可）
      //     注意：这里为了减少 spam，你可以自己加节流/去重逻辑。
      // -----------------------------
      const std::string region = classifyRegion(obj_pose.position.x, obj_pose.position.y);

      if (region != "UNKNOWN") {
        std_msgs::String obs;
        std::ostringstream ss;
        // 观测字符串格式：object_name,region_id,range_m
        const double dx0 = obj_pose.position.x - x0_;
        const double dy0 = obj_pose.position.y - y0_;
        const double r0  = std::sqrt(dx0*dx0 + dy0*dy0);

        ss << obj_name << "," << region << "," << r0;
        obs.data = ss.str();
        pub_obs_.publish(obs);
      }

      // -----------------------------
      // 3.2 写入知识库（只在“第一次近距离看到”该物体时做一次）
      //     目的：避免重复调用 /assert_knowledge
      // -----------------------------
      if (dist_to_robot <= vision_range_m_) {
        if (seen_objects_.find(obj_name) == seen_objects_.end()) {
          seen_objects_.insert(obj_name);

          world_percept_assig4::UpdateObjectList srv;
          // ⚠️ 注意：如果你的 srv 字段名不是 object_name，请在这里改成正确字段名
          srv.request.object_name = obj_name;

          if (client_assert_kb_.call(srv)) {
            ROS_INFO_STREAM("[percept] 近距离首次看到物体 -> 已写入KB: " << obj_name
                            << " (dist=" << dist_to_robot << ")");
          } else {
            ROS_WARN_STREAM("[percept] 调用 /assert_knowledge 失败，物体: " << obj_name);
          }
        }
      }
    }
  }

private:
  ros::NodeHandle nh_;

  // ROS 通信对象
  ros::Subscriber sub_model_states_;
  ros::Publisher  pub_obs_;
  ros::ServiceClient client_assert_kb_;

  // 参数
  std::string subs_topic_name_;
  std::string pub_obs_topic_name_;
  std::string srv_assert_knowledge_name_;
  std::string robot_model_name_;
  double vision_range_m_ = 1.1;
  double sector_max_range_m_ = 10.0;

  // 初始位姿（只记录一次）
  bool have_start_pose_ = false;
  double x0_ = 0.0;
  double y0_ = 0.0;
  double yaw0_ = 0.0;

  // 去重集合：记录已经“近距离首次看到并写入 KB”的物体
  std::unordered_set<std::string> seen_objects_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "percept_node");
  ros::NodeHandle nh("~");

  PerceptNode node(nh);
  ros::spin();
  return 0;
}