# Task2（Learning）闭环测试说明（Percept → Learning → Reasoning → Control）

本 README 对应你当前的最小可跑通版本：
- percept_node：发布观测 `/percept/observations`（std_msgs/String，格式 `object,sector,range`），并写入 KB（/assert_knowledge）
- learning_node：订阅观测 -> 写 CSV -> 调 scikit-learn(Python) 服务得到概率 -> 写入 KB（/update_likelihood）-> 从 reasoning 获取候选并按概率选最优 -> 调 control 执行动作
- reasoning_node：维护 Prolog KB；提供 /query_knowledge，并根据 KB 中的概率对候选排序
- ml_node.py：DecisionTreeClassifier 读取 CSV 训练/评估，并通过 /ml/train_predict 输出 8 扇区概率

> 注意：你现在使用的是“8 方向扇区 + 10m 半径”的占位区域划分。后续换真实书架/桌子/垃圾桶，只需要替换 percept 与 placeToSector 的映射逻辑。

---

## 1. 文件放置（建议做法：覆盖原文件）

把下列文件放到你的 ROS 包（例：`world_percept_assig4`）中：

- `src/percept_node.cpp`：使用 `percept_node_cn_full.cpp` 的内容覆盖
- `src/learning_node.cpp`：使用 `learning_node_scikit_cn_full.cpp` 的内容覆盖（或你当前最新版本）
- `src/reasoning_node.cpp`：使用本次提供的 `reasoning_node_scikit_cn_full.cpp` 覆盖
- `scripts/ml_node.py`：新增本次提供的 Python 文件，并 `chmod +x`
- `srv/TrainPredict.srv`：新增（learning_node 已 include）
- `srv/UpdateLikelihood.srv`：新增（reasoning_node 提供，learning_node 需要调用）

---

## 2. CMakeLists.txt / package.xml 必改项

### CMakeLists.txt
确保包含 `message_generation` 并生成 srv：

```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp rospy std_msgs message_generation
)

add_service_files(
  FILES
  TrainPredict.srv
  UpdateLikelihood.srv
  # 你原有的 srv 继续保留
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS roscpp rospy std_msgs message_runtime)
```

### package.xml
确保有：

```xml
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```

---

## 3. Python 依赖

确保 Python3 能 import：
- numpy
- pandas
- scikit-learn

例如：
```bash
pip3 install numpy pandas scikit-learn
```

---

## 4. 编译

在 catkin_ws 下：

```bash
catkin_make
source devel/setup.bash
```

---

## 5. 运行顺序（推荐）

### 5.1 启动仿真（Gazebo）与 control 等
按你原来的 launch 启动（保持不变）。

### 5.2 启动 reasoning_node
```bash
rosrun world_percept_assig4 reasoning_node
```

### 5.3 启动 percept_node
```bash
rosrun world_percept_assig4 percept_node
```

### 5.4 启动 Python 学习节点（scikit-learn）
```bash
rosrun world_percept_assig4 ml_node.py
```

### 5.5 启动 learning_node
```bash
rosrun world_percept_assig4 learning_node
```

---

## 6. 如何证明你完成了 Task2（Learning）

Task2 要求（作业说明）：
1) 至少一种学习方法（这里：DecisionTreeClassifier）
2) 至少一个在线工具（这里：scikit-learn + pandas）
3) 至少一种评估方式（这里：accuracy + confusion matrix）

你需要在视频/日志中展示以下证据链：

### 6.1 数据集在增长（学习证据）
每次找到 bowl 会追加一行到：
- `~/.ros/bowl_dataset.csv`

你可以打开验证：
```bash
tail -n 5 ~/.ros/bowl_dataset.csv
```

### 6.2 Python 节点输出评估指标（评估证据）
`ml_node.py` 会在 rosconsole 输出类似：

- `[ml] samples=XX acc(test)=0.75`
- `[ml] confusion_matrix(labels=[...])= ...`

这满足“evaluation performance”。

### 6.3 学习结果写入 KB（Learning → KB）
learning_node 每次决策都会调用 `/ml/train_predict`，得到 8 扇区概率，并逐个调用：
- `/update_likelihood`（写入 Prolog KB：likely_sector/3）

你可以用日志看到：
- `[learning] ML probs: E=... NE=...`
- `[reasoning] return candidates sorted by learned prob ...`

### 6.4 决策行为变化（Learning → Reasoning → Decision）
在训练样本较少时，reasoning 返回的候选排序接近默认；样本增多后：
- `/query_knowledge` 会按学习到的概率排序候选地点
- learning_node 会更倾向于选择高概率区域（例如 big_table 对应扇区）

你可以录视频对比：
- Episode 前期：探索顺序更随机
- Episode 后期：明显更快/更偏向高概率区域

---

## 7. 常见问题排查

1) **/ml/train_predict 调用失败**
- 确认 `rosrun world_percept_assig4 ml_node.py` 已在运行
- 确认 srv 已生成：`rossrv list | grep TrainPredict`

2) **/update_likelihood 不存在**
- 确认 reasoning_node 已启动
- 确认 srv 已生成：`rossrv list | grep UpdateLikelihood`

3) **CSV 一直不增长**
- 确认 percept_node 正在发布：`rostopic echo /percept/observations`
- 确认目标物体名与 learning_node 参数一致（默认 bowl）

4) **候选地点排序没有变化**
- 先看 `ml_node.py` 的概率是否不是均匀的（样本太少会均匀）
- 再看 reasoning_node 是否能读到 likely_sector（日志会提示 sorted）
- 再看 placeToSector 映射是否与候选地点名称匹配（big_table/small_table/...）
