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