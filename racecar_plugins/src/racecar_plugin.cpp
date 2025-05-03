#include <memory>
#include <racecar_plugins/racecar_plugin.hpp>

#define GREEN "\033[1;32m"
#define RESET "\033[0m"

namespace gazebo
{
    RacecarPlugin::RacecarPlugin(){
        std::cout<<GREEN<<"Initialized Racecar Plugin"<<RESET<<std::endl;
    }
    RacecarPlugin::~RacecarPlugin(){
        _update_connection.reset();
        std::cout<<GREEN<<"Destroyed Racecar Plugin"<<RESET<<std::endl;
    }
    void RacecarPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
    {
        // Initialize the ROS Node
        _rosnode = gazebo_ros::Node::Get(sdf); 
        
        // Bind the "update" function. This function will be called on every iteration of the simulation
        _update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&RacecarPlugin::update, this)); 
        
        // Get the model. For us, it is the racecar.
        _model = model;

        // Initialize the publisher
        _pub_odom =
            _rosnode->create_publisher<nav_msgs::msg::Odometry>("/odom", 1);
        _sub_pose = _rosnode->create_subscription<geometry_msgs::msg::Pose>(
            "/pose/bicycle", 10, std::bind(&RacecarPlugin::PoseCallback, this, std::placeholders::_1));
        
    }
    void RacecarPlugin::update() {
        _offset = _model->WorldPose();
        publishOdom();
        setPose();
        
    }
    void RacecarPlugin::publishOdom(){
        
        nav_msgs::msg::Odometry _odom;
        _odom.pose.pose.position.x = _offset.Pos()[0];
        _odom.pose.pose.position.y = _offset.Pos()[1];
        _odom.pose.pose.position.z = _offset.Pos()[2];
        
        std::vector<double> euler = {0.0, 0.0, _offset.Rot().Yaw()}; 
        std::vector<double> quat = ToQuaternion(euler);
        _odom.pose.pose.orientation.x = quat[0];
        _odom.pose.pose.orientation.y = quat[1];
        _odom.pose.pose.orientation.z = quat[2];
        _odom.pose.pose.orientation.w = quat[3];

        ignition::math::Vector3 _linear_vel = _model->WorldLinearVel();
        _odom.twist.twist.linear.x = _linear_vel[0];
        _odom.twist.twist.linear.y = _linear_vel[1];
        _odom.twist.twist.linear.z = _linear_vel[2];

        ignition::math::Vector3 _angular_vel = _model->WorldAngularVel();
        _odom.twist.twist.angular.x = _angular_vel[0];
        _odom.twist.twist.angular.y = _angular_vel[1];
        _odom.twist.twist.angular.z = _angular_vel[2];

        _pub_odom->publish(_odom);
    }
    std::vector<double> RacecarPlugin::ToQuaternion(std::vector<double> &euler) {
        // Abbreviations for the various angular functions
        double cy = cos(euler[0] * 0.5);
        double sy = sin(euler[0] * 0.5);
        double cp = cos(euler[1] * 0.5);
        double sp = sin(euler[1] * 0.5);
        double cr = cos(euler[2] * 0.5);
        double sr = sin(euler[2] * 0.5);
      
        std::vector<double> q;
        q.reserve(4);
        q[0] = cy * cp * sr - sy * sp * cr;  // x
        q[1] = sy * cp * sr + cy * sp * cr;  // y
        q[2] = sy * cp * cr - cy * sp * sr;  // z
        q[3] = cy * cp * cr + sy * sp * sr;  // w
      
        return q;
    }
    double RacecarPlugin::YawFromQuat(){
        std::vector<double> euler;
        euler.reserve(3);
        std::vector<double> quat = {_pose.orientation.x, _pose.orientation.y, _pose.orientation.z, _pose.orientation.w};
        double sinr_cosp = 2 * (quat[3] * quat[0] + quat[1] * quat[2]);
        double cosr_cosp = 1 - 2 * (quat[0] * quat[0] + quat[1] * quat[1]);
        euler[0] = atan2(sinr_cosp, cosr_cosp);
      
        // pitch (y-axis rotation)
        double sinp = 2 * (quat[3] * quat[1] - quat[2] * quat[0]);
        if (fabs(sinp) >= 1)
            euler[1] = copysign(M_PI / 2, sinp);  // use 90 degrees if out of range
        else
            euler[1] = asin(sinp);
      
        // yaw (z-axis rotation)
        double siny_cosp = 2 * (quat[3] * quat[2] + quat[0] * quat[1]);
        double cosy_cosp = 1 - 2 * (quat[1] * quat[1] + quat[2] * quat[2]);
        euler[2] = atan2(siny_cosp, cosy_cosp);
      
        return euler[2];
    }    
    void RacecarPlugin::setPose(){
        double yaw = YawFromQuat();
        // Set the pose of the model
        _model->SetWorldPose(ignition::math::Pose3d(_pose.position.x, _pose.position.y, _offset.Pos()[2], 0, 0, yaw));
    }

    void RacecarPlugin::PoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
        _pose.position.x = msg->position.x;
        _pose.position.y = msg->position.y;
        _pose.position.z = _offset.Pos()[2]+1.0;
        _pose.orientation.x = msg->orientation.x;
        _pose.orientation.y = msg->orientation.y;
        _pose.orientation.z = msg->orientation.z;
        _pose.orientation.w = msg->orientation.w;
    }
    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(RacecarPlugin)
}
