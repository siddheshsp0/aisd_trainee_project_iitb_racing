#ifndef RACECAR_PLUGIN_HPP_
#define RACECAR_PLUGIN_HPP_
 
#include "rclcpp/rclcpp.hpp"

#include <gazebo_ros/node.hpp>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/transport/transport.hh>
 
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
 namespace gazebo {

    class RacecarPlugin : public gazebo::ModelPlugin
    {
        public:
        RacecarPlugin();
        ~RacecarPlugin() override;
        void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr _sdf) override;
        void update();

        private:
        ignition::math::Pose3d _offset;
        gazebo::event::ConnectionPtr _update_connection;
        std::shared_ptr<rclcpp::Node> _rosnode;
        gazebo::physics::ModelPtr _model;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _pub_odom;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr _sub_pose;
        const float _eval_rate=0.01;
        geometry_msgs::msg::Pose _pose;
        void publishOdom();
        void setPose();
        std::vector<double> ToQuaternion(std::vector<double> &euler);
        double YawFromQuat();
        void PoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
    };

 
 #endif  // RACECAR_PLUGIN_HPP_
}  // namespace gazebo_plugins 