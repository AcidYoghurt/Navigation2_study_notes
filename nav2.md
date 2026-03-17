# 1 导航概念

## 1.1 导航服务器

> NAV2包含了四个**动作服务器（ROS2中的Action通讯）**，分别是：`规划器`、`平滑器`、`控制器`和`恢复服务器`

- NAV2 四大导航服务器的**整体流程图**：

  ```mermaid
  flowchart LR
      Goal[目标点] --> Planner[规划器服务器]
      Planner --> Smoother[平滑器服务器]
      Smoother --> Controller[控制器服务器]
      Controller -->|成功| CmdVel[发布 cmd_vel 速度指令]
      Controller -->|失败| Recovery[恢复服务器]
      Recovery -->|自救成功| Planner
  
  ```

### 规划器

- **负责**：从当前位置到目标点生成一条**全局路径**

- **输入**：
  - 当前机器人位姿（map系）
  - 目标位姿
  - 全局代价地图（global costmap）

- **输出**：`nav_msgs/Path话题`

- **官方提供的规划器**：

  见：https://docs.nav2.org/setup_guides/algorithm/select_algorithm.html

  | 插件名称               | 核心算法                          | 是否考虑运动学 | 路径特点             | 适合底盘                       | 典型使用场景              | 难度 / 推荐度                  |
  | ---------------------- | --------------------------------- | -------------- | -------------------- | ------------------------------ | ------------------------- | ------------------------------ |
  | **NavFn Planner**      | A* / Dijkstra                     | ❌ 不考虑       | 网格路径，可能不平滑 | 差速 / 全向 / 腿式             | 传统室内导航              | ⭐ 简单 / 老牌稳定（偏 legacy） |
  | **SmacPlannerHybrid**  | Hybrid-A*（Dubins / Reeds-Shepp） | ✅ 考虑         | 可执行、带转弯半径   | Ackermann / 差速 / 全向 / 腿式 | 自动驾驶、AGV、车类机器人 | ⭐⭐⭐⭐ 强烈推荐（车类首选）      |
  | **SmacPlanner2D**      | 优化 A*（4/8邻域）                | ❌ 不考虑       | 比 NavFn 更快更平滑  | 差速 / 全向 / 腿式             | 室内移动机器人            | ⭐⭐⭐⭐ 通用首选                  |
  | **SmacPlannerLattice** | State Lattice + Motion Primitives | ✅ 强运动学     | 高真实度、可自定义   | 任意底盘 / 自定义机器人        | 研究项目、复杂底盘        | ⭐⭐⭐ 功能最强 / 配置复杂        |
  | **ThetaStarPlanner**   | Theta*（Any-angle）               | ❌ 不考虑       | 路径更直、更自然     | 差速 / 全向                    | 仓储机器人、室内导航      | ⭐⭐⭐ 简单好用                   |

- yaml文件配置示例：

  ```yaml
  planner_server:
    ros__parameters:
      planner_plugins: ['GridBased']
      GridBased:
        plugin: 'nav2_navfn_planner::NavfnPlanner' # 在Iron及更老的版本，使用"/"而不是"::"
  ```

  

### 平滑器

- **负责**：美化路径，把那些直角一样的路径变成曲线

- 为什么需要这个**平滑器**呢？
  - 因为**规划器输出的路径**往往会有**直角转弯**、**抖动**，甚至不符合**机器人的运动学**
  - 那么这个时候我们就要改正这些路径，使得机器人可以稳定行走
- **输入**：原始的`nav_msgs/Path话题`
- **输出**：优化后的`nav_msgs/Path话题`

- **官方提供的控制器**：

  见：https://docs.nav2.org/setup_guides/algorithm/select_algorithm.html

- **官方提供的插件**：

  | 插件名称                    | 核心算法 / 特点                                | 支持规划器类型                | 典型用途                   | 难度 / 推荐度        |
  | --------------------------- | ---------------------------------------------- | ----------------------------- | -------------------------- | -------------------- |
  | **Simple Smoother**         | 简单路径平滑，针对不可行路径（如 2D 网格）     | 2D / 基本规划器               | 去除尖角，平滑路径         | ⭐⭐⭐ 简单好用         |
  | **Constrained Smoother**    | 约束优化平滑（考虑最小转弯半径、障碍物距离等） | 适合 Ackermann / 非全向机器人 | 高级路径优化，运动学约束   | ⭐⭐⭐⭐ 高性能 / 高精度 |
  | **Savitzky-Golay Smoother** | 数字信号处理滤波，去除路径噪声                 | 2D / 全向 / 差速              | 去除路径噪声，保持轨迹形状 | ⭐⭐⭐ 实用、轻量       |



### 控制器

- **负责**：根据路径实时计算**速度指令**

- **输入**：
  
  - 当前 odom 位姿
  - 局部代价地图
  - 全局路径
  
- **输出**：`cmd_vel`话题，该话题包括了控制器计算出来的**线速度+角速度**，然后我们可以把这个`cmd_vel`话题中的线速度和角速度直接发给电控

- **官方提供的控制器**：

  见：https://docs.nav2.org/setup_guides/algorithm/select_algorithm.html

  | 插件名称                         | 核心算法 / 特点                           | 支持底盘类型                   | 典型用途 / 场景               | 难度 / 推荐度                 |
  | -------------------------------- | ----------------------------------------- | ------------------------------ | ----------------------------- | ----------------------------- |
  | **DWB Controller**               | 高度可配置的 DWA（动态窗口法） + 插件接口 | 差速 / 全向 / 腿式             | 动态避障、通用导航            | ⭐⭐⭐ 经典默认                  |
  | **TEB Controller**               | MPC-like / Timed Elastic Band             | Ackermann / 差速 / 全向 / 腿式 | 优化路径时间、动态避障        | ⭐⭐⭐⭐ 高性能 / 调参多          |
  | **Regulated Pure Pursuit (RPP)** | 纯追踪算法变体 + 自适应调节               | Ackermann / 差速 / 腿式        | 服务 / 工业机器人精确路径跟踪 | ⭐⭐⭐⭐ 室内机器人首选           |
  | **MPPI Controller**              | 预测式 MPC，模块化成本函数                | 差速 / 全向 / Ackermann        | 高动态环境、高性能机器人      | ⭐⭐⭐⭐⭐ 新一代强力控制器        |
  | **Rotation Shim Controller**     | 路径跟踪前旋转到目标朝向                  | 差速 / 全向                    | 与主控制器结合使用            | ⭐⭐⭐ 高速 / 精准路径前置控制   |
  | **Graceful Controller**          | 基于姿态跟随控制法生成平滑轨迹            | 差速 / 全向 / 腿式             | 生成平滑运动轨迹              | ⭐⭐⭐ 室内/服务机器人优选       |
  | **Vector Pursuit Controller**    | Vector Pursuit 算法，高速精确路径跟踪     | 差速 / Ackermann / 腿式        | 高速路径跟踪或急转弯          | ⭐⭐⭐ 高速机器人 / 算力有限场景 |

- yaml文件配置示例：

  ```yaml
  controller_server:
    ros__parameters:
      controller_plugins: ["FollowPath"]
      FollowPath:
         plugin: "dwb_core::DWBLocalPlanner"
  ```

  

### 恢复服务器

- **负责**：当导航失败时（如机器人卡墙了）尝试**自救**
- **什么时候触发？**
  - 控制器算不出速度
  - 被障碍堵住
  - costmap 异常
  - 机器人卡住

- **常见的恢复行为**：
  - **ClearCostmap**：清理代价地图
  - **Spin**：原地旋转
  - **Backup**：后退
  - **Wait**：等待
  - **AssistedTeleop**：请求人工干预



## 导航中的状态估计

- 在学习`NAV2`中我们经常会有一个不太清楚的地方：谁在维护TF树？谁在使用TF树？

  **答**：

  - TF 树由机器人系统中的各类节点（如定位、里程计、robot_state_publisher 等）共同维护；
  - **Nav2 本身只使用 TF，不负责维护 TF树。**

- `NAV2`需要我们去维护的 **TF树** 是 `map` -> `odom` -> `base_link` -> `[sensor frames]`

  > [!NOTE]
  >
  > `map`：地图坐标系（全局坐标系），用于表示 **机器人在地图中的绝对位置**
  >
  > `odom`：局部里程计坐标系，由机器人运动积分产生，用于描述连续平滑的相对运动。因为机器人会**在每次运动时**打滑，导致机器人运动的目标位姿与**里程计(odom系)**计算出来的结果不一样，所以就有了`map -> odom`表示`定位系统`估计机器人打滑导致的误差并修正，这就导致我们在rviz中以map系为参考系时看到的odom系的位置会变化，也就是说
  >
  > ```
  > 机器人在map的实际位置 = 里程计估计的机器人运动的值值(odom->base_link) + 误差值(map->odom)
  > ```
  >
  > `base_link`：机器人底盘坐标系，通常作为机器人主体参考坐标
  >
  > `[sensor frames]`：机器人上的传感器坐标系（如激光雷达、相机、IMU 等）

  - **全局定位系统**（如GPS、SLAM、动作捕捉）提供： `map` -> `odom` 的转换，表示**局部里程计坐标系在全局地图中的偏移量**

    官方提供了两种全局定位系统：

    - `amcl`：或许你会听说过NAV2自带的`amcl`，amcl就是一种全局定位技术，是基于粒子滤波器的自适应蒙特卡洛实现的。amcl只负责定位而不负责建图，只适用于已有**静态地图场景**

    - `slam_toolbox`：slam_toolbox是一个独立的SLAM组件，专门用于给机器人进行**建图**和**定位**的。因为slam_toolbox是跟ros2紧密结合的，所以一般在实际运用中我们会使用`slam_toolbox`来进行机器人**定位**（即发布map->odom），然后使用NAV2来进行**导航**

  - **里程计系统**提供：`odom` -> `base_link` 的转换，表示机器人在odom坐标系的实时位姿。
    
    - 例如我们用的是**雷达**，就需要我们提供`base_link` -> `lidar`的转换
    - 如果用的是**深度相机**，就需要我们提供`base_link` -> `depth_camera`的转换
    - 如果用的是**轮式里程计**，需要我们提供`base_link` -> `imu`的转换



## 环境表示

- **环境表示**就是机器人能看到的`地图系统`

- **环境表示**有**两个作用**：

  - 把各种`传感器数据`合成一个统一空间（激光雷达、相机、深度、雷达、声呐…）

  - 为`规划器`和`控制器`提供数据

### 代价地图图层(Costmap Layers)

- **代价地图图层**是**代价地图**的组成成分，**一个代价地图**由**多个代价地图图层**组成

- 每个**代价地图图层** 都是一个 **pluginlib 插件**
  - **激光雷达layer**：用激光雷达生成障碍
  - **深度相机layer**：也是依靠深度相机的数据来生成障碍
  - **静态地图layer**：加载建好的地图或SLAM实时建的图
  - **膨胀层inflation layer**：把障碍周围变成“危险区”

- 一个**costmap layer** = 一个**往 costmap 里写数据的插件**，每个layer只负责一件事，如：
  - 把激光雷达数据变成障碍
  - 把地图加载进来
  - 在障碍周围生成代价（变危险）
  - 把视觉检测结果写进去

- **官方提供的插件**：

  > [!NOTE]
  >
  > 代价地图图层不只选择一个，可以选择很多个代价地图图层

  | 插件名称                         | 主要功能               | 数据来源        | 维度 | 典型用途           | 推荐程度 / 备注  |
  | -------------------------------- | ---------------------- | --------------- | ---- | ------------------ | ---------------- |
  | **Static Layer（静态地图层）**   | 加载静态地图占用信息   | 地图服务器      | 2D   | 室内导航基础地图   | ⭐⭐⭐⭐⭐ 几乎必用   |
  | **Obstacle Layer（动态障碍层）** | 根据2D激光维护动态障碍 | LaserScan       | 2D   | 动态避障           | ⭐⭐⭐⭐⭐ 常规必备   |
  | **Inflation Layer（膨胀层）**    | 障碍物膨胀安全距离     | Costmap数据     | 2D   | 安全缓冲           | ⭐⭐⭐⭐⭐ 必备安全层 |
  | **Voxel Layer（3D体素层）**      | 持久3D体素地图         | 深度相机 / 激光 | 3D   | 高度信息、立体障碍 | ⭐⭐⭐⭐ 3D感知常用  |
  | **Range Layer**                  | 处理range传感器数据    | 超声波 / 红外   | 2D   | 近距离传感器       | ⭐⭐⭐ 特定传感器用 |
  | **Spatio-Temporal Voxel Layer**  | 带时间衰减的3D体素地图 | 深度 / LiDAR    | 3D   | 动态环境建图       | ⭐⭐⭐⭐ 新一代3D层  |
  | **Non-Persistent Voxel Layer**   | 非持久3D占用栅格       | 深度 / LiDAR    | 3D   | 临时障碍检测       | ⭐⭐⭐ 实时障碍     |
  | **Denoise Layer**                | 去除孤立噪声障碍       | 任意传感器      | 2D   | 过滤误检           | ⭐⭐⭐⭐ 很实用      |
  | **Plugin Container Layer**       | 组合多个layer          | 其他layer       | —    | 模块化组合         | ⭐⭐ 高级配置用    |



### 代价地图(Costmap)

> Nav2 现在默认用的环境表示就是**Costmap**
>
> **代价地图**本质是一个`2D网络地图`

Costmap中每一个格子都有一个代价值，如：

| 状态     | 含义                 |
| -------- | -------------------- |
| unknown  | 不知道有没有东西     |
| free     | 可通行               |
| occupied | 有障碍               |
| inflated | 靠近障碍（危险区域） |

- `规划器服务器`就是在这个网格地图上 **搜索路径**

- `控制器服务器`在局部区域 **采样运动**

  

### 代价地图过滤器(Costmap Filters)

> `代价地图过滤器`不是用传感器改变地图，而是**用`规则地图`改变机器人的行为**。

- 代价地图过滤器的输入不是现实世界，而是 **人为定义的语义规则**
- 简单的说，`代价地图过滤器`就是在`代价地图`上标注出：
  - 机器人永远不会进入的禁止/安全区域。
  - 速度限制区域。进入这些区域的机器人的最大速度将受限制。
  - 机器人在工业环境和仓库中移动的优选路径。

- `代价地图过滤器`与`代价地图层`的本质区别

  | 维度     | Costmap Layer  | Costmap Filter |
  | -------- | -------------- | -------------- |
  | 信息来源 | 现实世界       | 人为规则       |
  | 改什么   | 环境模型       | 行为策略       |
  | 本质     | 感知融合       | 策略注入       |
  | 目标     | 知道哪里有东西 | 决定怎么行动   |

- **官方提供的插件**：

  | 插件名称           | 作者              | 功能描述                       | 数据作用 / 用途                                | 推荐程度 / 备注            |
  | ------------------ | ----------------- | ------------------------------ | ---------------------------------------------- | -------------------------- |
  | **Keepout Filter** | Alexey Merzlyakov | 管理禁止区 / 安全区 / 优先通道 | 在地图上定义不可通行区域或偏好通行路线         | ⭐⭐⭐⭐ 工业 / 服务机器人常用 |
  | **Speed Filter**   | Alexey Merzlyakov | 限制区域内最大速度             | 控制机器人在敏感区域减速（如拐角或人员区）     | ⭐⭐⭐⭐ 高速或工业场景重要    |
  | **Binary Filter**  | Alexey Merzlyakov | 二值掩码触发行为               | 将特定区域标记为 True/False 用于触发事件或行为 | ⭐⭐⭐ 灵活扩展用，场景驱动   |



### 环境表示的其他形式

还存在其他形式的环境表示。这些包括：

- 梯度地图，类似于成本地图，但表示表面梯度以检查可通过性
- 3D成本地图，表示三维空间，但也需要进行三维规划和碰撞检查。
- 网格地图，类似于梯度地图，但包含多个角度的表面网格
- “向量空间”，利用传感器信息并使用机器学习来检测个体物品和位置以进行跟踪，而不是缓冲离散点。



# 2 导航进阶

## 2.1 自定义规划器

### 2.1.1 自定义规划器的基本概念

- **路径规划器的任务**是基于给定的机器人`初始位姿`，`目标位姿`和`环境地图`来计算出一条可以行走的`路径`

- 三个**基本概念**：

  - `位姿`**(初始位姿, 目标位姿)**：geometry_msgs/msg/PoseStamped

  - `路径`：nav_msgs/msg/Path

    消息格式：

    ```bash
    # An array of poses that represents a Path for a robot to follow.
    
    # Indicates the frame_id of the path.
    std_msgs/Header header
    	builtin_interfaces/Time stamp
    		int32 sec
    		uint32 nanosec
    	string frame_id
    
    # Array of poses to follow.
    geometry_msgs/PoseStamped[] poses		# 我们可以看到，路径是由多个点组成的
    	std_msgs/Header header
    		builtin_interfaces/Time stamp
    			int32 sec
    			uint32 nanosec
    		string frame_id
    	Pose pose
    		Point position
    			float64 x
    			float64 y
    			float64 z
    		Quaternion orientation
    			float64 x 0
    			float64 y 0
    			float64 z 0
    			float64 w 1
    ```

  - `占据珊格地图`：nav_msgs/msg/OccupancyGrid

    消息格式：

    ```bash
    # This represents a 2-D grid map
    std_msgs/Header header
    	builtin_interfaces/Time stamp
    		int32 sec
    		uint32 nanosec
    	string frame_id
    
    # MetaData for the map
    MapMetaData info
    	builtin_interfaces/Time map_load_time
    		int32 sec
    		uint32 nanosec
    	float32 resolution				# 表示像素，在导航中，默认一个像素代表0.05m，1m有20个像素点
    	uint32 width							# 地图的宽(m)
    	uint32 height							# 地图的高(m)
    	geometry_msgs/Pose origin	# 地图的原点位姿
    		Point position					# 地图的坐标
    			float64 x
    			float64 y
    			float64 z
    		Quaternion orientation	# 地图的姿态（默认是不旋转）
    			float64 x 0
    			float64 y 0
    			float64 z 0
    			float64 w 1
    
    # The map data, in row-major order, starting with (0,0).
    # Cell (1, 0) will be listed second, representing the next cell in the x direction.
    # Cell (0, 1) will be at the index equal to info.width, followed by (1, 1).
    # The values inside are application dependent, but frequently,
    # 0 represents unoccupied, 1 represents definitely occupied, and
    # -1 represents unknown.
    int8[] data									# 整个地图的原始数据，每个数据的取值是0~256
    ```


- **坐标转换**



### 2.1.2 搭建规划器插件框架

我们的示例插件继承自基类 `nav2_core::GlobalPlanner`。基类提供了 5 个纯虚拟方法来实现规划器插件。该插件将由规划器服务器用于计算轨迹。让我们更多地了解编写规划器插件所需的方法。

| **虚拟方法** | **方法描述**                                                 | **需要重写？** |
| ------------ | ------------------------------------------------------------ | -------------- |
| configure()  | 该方法在规划器服务器进入 on_configure 状态时调用。理想情况下，这个方法应该执行ROS参数的声明和规划器成员变量的初始化。该方法接受4个输入参数：父节点的共享指针、规划器名称、tf缓冲指针和成本地图的共享指针。 | 是             |
| activate()   | 当规划器服务器进入 on_activate 状态时调用该方法。理想情况下，此方法应该实现在规划器进入活动状态之前必要的操作。 | 是             |
| deactivate() | 该方法在规划器服务器进入 on_deactivate 状态时调用。理想情况下，这个方法应该实现在规划器进入非活动状态之前必要的操作。 | 是             |
| cleanup()    | 当规划器服务器进入on_cleanup状态时调用此方法。理想情况下，此方法应清理为规划器创建的资源。 | 是             |
| createPlan() | 当规划器服务器要求指定起始和目标姿态的全局路径时调用此方法。此方法返回携带全局路径的`nav_msgs::msg::Path`。此方法接受两个输入参数：起始姿态和目标姿态。 | 是             |

- 项目结构

  ```bash
  nav2_custom_planner
    ├── CMakeLists.txt
    ├── include
    │   └── nav2_custom_planner
    │       └── nav2_custom_planner.hpp		# 插件实现类的代码声明（继承抽象类的类，这里的抽象类是nav2_core::GlobalPlanner）
    ├── package.xml
    ├── plugin.xml												# 插件描述文件
    └── src
        └── nav2_custom_planner.cpp				# 插件实现类的代码实现
  ```

- 定义`插件的实现类`：**nav2_custom_planner.hpp**

  ```c++
  #pragma once
  #include <nav2_core/global_planner.hpp>
  
  namespace nav2_planner_system
  {
      // 自定义导航规划器类
      class CustomPlanner : public nav2_core::GlobalPlanner
      {
      public:
          CustomPlanner() = default;
          ~CustomPlanner() override = default;
  
          // 插件配置方法
          void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
              std::string name,
              std::shared_ptr<tf2_ros::Buffer> tf,
              std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  
          // 插件清理方法
          void cleanup() override;
  
          // 插件激活方法
          void activate() override;
  
          // 插件停用方法
          void deactivate() override;
  
          // 为给定的起始和目标位姿创建路径的方法
          nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped& start,
              const geometry_msgs::msg::PoseStamped& goal,
              std::function<bool()> cancel_checker) override;
      private:
          // 坐标变换缓存指针，用于查询坐标关系
          std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
          // 节点指针
          nav2_util::LifecycleNode::SharedPtr node_;
          // 全局代价地图
          nav2_costmap_2d::Costmap2D *costmap_2d_;
          // 全局代价地图坐标系
          std::string global_frame_,name_;
          // 插值分辨率
          double interpolation_resolution_;
      };
  }
  ```

- `插件的实现类`的**具体代码**实现：**nav2_custom_planner.cpp**

  ```c++
  #include<nav2_custom_planner/nav2_custom_planner.hpp>
  
  namespace nav2_planner_system
  {
      void CustomPlanner::configure(
          const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
          std::string name,
          std::shared_ptr<tf2_ros::Buffer> tf,
          std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
      {
          tf_buffer_ = tf;
          node_ = parent.lock();
          name_ = name;
          costmap_2d_ = costmap_ros->getCostmap();
          global_frame_ = costmap_ros->getGlobalFrameID();
  
          // 获取插值分辨率的参数
          node_->declare_parameter(name_+".interpolation_resolution",0.1);
          node_->get_parameter(name_+".interpolation_resolution",interpolation_resolution_);
      }
  
      void CustomPlanner::cleanup()
      {
          RCLCPP_INFO(node_->get_logger(),"正在清理类型为 CustomPlanner 的插件 %s",name_.c_str());
      }
  
      void CustomPlanner::activate()
      {
          RCLCPP_INFO(node_->get_logger(),"正在激活类型为 CustomPlanner 的插件 %s",name_.c_str());
      }
  
      void CustomPlanner::deactivate()
      {
          RCLCPP_INFO(node_->get_logger(),"正在停用类型为 CustomPlanner 的插件 %s",name_.c_str());
      }
  
      nav_msgs::msg::Path CustomPlanner::createPlan(const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal, std::function<bool()> cancel_checker)
      {
  
      }
  }
  
  // 导出插件
  #include <pluginlib/class_list_macros.hpp>
  PLUGINLIB_EXPORT_CLASS(nav2_planner_system::CustomPlanner,nav2_core::GlobalPlanner)
  ```

- `插件描述文件`

  ```xml
  <!-- 声明一个插件动态库（对应 CMake 里的 add_library 名字），后面给出 CMakeLists.txt -->
  <library path="nav2_custom_planner">
  
      <!-- 声明一个可被 pluginlib 加载的类 -->
      <!-- 参数： -->
      <!-- name：插件唯一ID：代码里 createSharedInstance() 用的字符串，这个字符串可以任意的，可以是AStarPlanner，也可以是plan_system/AStarPlanner(一般使用这种) -->
      <!-- type：真正的 C++ 类名（必须包含完整 namespace） -->
      <!-- base_class_type：插件基类（必须和 ClassLoader 的基类一致） -->
      <class
              name="nav2_planner_system/CustomPlanner"
              type="nav2_planner_system::CustomPlanner"
              base_class_type="nav2_core::GlobalPlanner">
  
          <!-- 描述信息：给人看的，不影响程序运行 -->
          <description>
              自定义nav2规划器
          </description>
      </class>
  
  </library>
  ```

- `CMakeLists.txt`关键代码

  ```cmake
  # 导出库
  ament_auto_add_library(
          nav2_custom_planner           # 【参数1】库的名字（CMake target 名）
          SHARED                        # 【参数2】库的类型
          src/nav2_custom_planner.cpp   # 【参数3】库的源文件列表
  )
  
  # 导出插件描述文件
  # 参数1：功能包的名字
  # 参数2：插件描述文件的位置
  pluginlib_export_plugin_description_file(nav2_custom_planner plugin.xml)
  ```

  

### 2.1.3 实现自定义规划算法

这里采用最简单的`直线规划策略`，当收到规划请求时，直接生成一个从当前位置到目标位置的直线路径，同时在规划时会判断路径上是否有障碍物，如果存在则抛出异常，表示规划失败

 





