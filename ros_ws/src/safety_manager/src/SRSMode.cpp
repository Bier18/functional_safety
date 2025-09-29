#include "safety_manager/SafetyTools.hpp"
#include "rclcpp/rclcpp.hpp"
#include "safety_msgs/msg/positions.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include <cmath>

namespace functional_safety
{
  class SRSMode : public functional_safety::SafetyTools
  {
  public:

      void initialize(const rclcpp::Node::SharedPtr & node) override
      {
          node_ = node;
          emergency_pub_ = node_->create_publisher<std_msgs::msg::Bool>("emergency_msg",10);
          human_presence_sub_ = node->create_subscription<safety_msgs::msg::Positions>(
            "human_monitoring",10,std::bind(&SRSMode::humanDetectedCallback,this,std::placeholders::_1)
          );
          emergency_active_ = false;
          RCLCPP_INFO(node_->get_logger(), "[SRSMode] Initialized SRS safety mode.");
      }

      void stop() override
      {
        RCLCPP_WARN(node_->get_logger(), "[SRSMode] Emergency Stop activated!");
        emergency_active_ = true;
        //crea il messaggio di stop e lo pubblica
        std_msgs::msg::Bool stop_msg;
        stop_msg.data = true;
        emergency_pub_->publish(stop_msg);
      }

      void pause() override
      {
          return;
      }

      void resume() override
      {
        emergency_active_ = false;
        //crea il messaggio di ripartenza e lo pubblica
        std_msgs::msg::Bool go_msg;
        go_msg.data = true;
        emergency_pub_->publish(go_msg);
      }

      void shutdown() override
      {
        emergency_active_ = false;

        if(human_presence_sub_)
        {
            human_presence_sub_.reset();
            RCLCPP_INFO(node_->get_logger(), "[SRSMode] Human presence subscription shut down.");
        }
        if(emergency_pub_){
            emergency_pub_.reset();
            RCLCPP_INFO(node_->get_logger(),"[SRSMode] Emergency publisher shut down.");
        }
        RCLCPP_WARN(node_->get_logger(), "[SRSMode] Shutdown completed.");
      }

      void checkVelocityLimits(const std::shared_ptr<geometry_msgs::msg::TwistWithCovariance> /*msg*/) override
      {
          return;
      }


      void checkTorqueLimits() override
      {
          return;
      }

      void monitorSensors() override
      {
          return;
      }

      void diagnose() override
      {
          return;
      }

      void safeMoveTo() override
      {
          return;
      }

      void trajectoryCheck() override
      {
          return;
      }

      void overrideControl() override
      {
          return;
      }

      void logEvent() override
      {
          return;
      }

      void alertOperator() override
      {
          RCLCPP_WARN(node_->get_logger(), "[SRSMode] Operator alert triggered!");
          return;
      }

      void getFaultHistory() override
      {
          return;
      }

      void set_safety_params(double param) override
      {
        min_distance_ = param;
      }

  private:
      rclcpp::Node::SharedPtr node_;
      bool emergency_active_;
      rclcpp::Subscription<safety_msgs::msg::Positions>::SharedPtr human_presence_sub_;
      rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_pub_;
      double min_distance_;

      void humanDetectedCallback(const safety_msgs::msg::Positions::SharedPtr msg)
      {
        //robot pose with uncertainty
        double rx = msg->robot_pose.pose.position.x;
        double ry = msg->robot_pose.pose.position.y;
        double rz = msg->robot_pose.pose.position.z;
        double srx = msg->robot_pose.covariance[0];
        double sry = msg->robot_pose.covariance[7];
        double srz = msg->robot_pose.covariance[14];
        //operator pose with uncertainty
        double ox = msg->op_pose.pose.position.x;
        double oy = msg->op_pose.pose.position.y;
        double oz = msg->op_pose.pose.position.z;
        double sox = msg->op_pose.covariance[0];
        double soy = msg->op_pose.covariance[7];
        double soz = msg->op_pose.covariance[14];
        //poses modules with uncertainty
        double r_pose = std::sqrt(rx*rx + ry*ry + rz*rz);
        double o_pose = std::sqrt(ox*ox + oy*oy + oz*oz);
        double sigma_o = 0.0;
        double sigma_r = 0.0;
        if(r_pose > 1e-6){
            sigma_r = std::sqrt((rx*rx*srx + ry*ry*sry + rz*rz*srz)/(r_pose*r_pose));
        }
        if(o_pose > 1e-6){
            sigma_o = std::sqrt((ox*ox*sox + oy*oy*soy + oz*oz*soz)/(o_pose*o_pose));
        }
        //distance with uncertainty
        double rob_op_distance_x = rx-ox;
        double rob_op_distance_y = ry-oy;
        double rob_op_distance_z = rz-oz;
        double rob_op_distance = std::sqrt(rob_op_distance_x*rob_op_distance_x + rob_op_distance_y*rob_op_distance_y + rob_op_distance_z*rob_op_distance_z);
        //worst case distance
        double k = 2.0; //confidence level
        rob_op_distance = rob_op_distance + k * (sigma_o + sigma_r);
        if (!emergency_active_ && rob_op_distance <= min_distance_)
        {
            RCLCPP_WARN(node_->get_logger(), "[SRSMode] Human detected! Triggering SRS stop.");
            stop();
        }else if (emergency_active_ && rob_op_distance > min_distance_){
            RCLCPP_WARN(node_->get_logger(), "[SRSMode] Human stepped away! Triggering SRS go.");
            resume();
        }
      }
  };

} // namespace functional_safety

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(functional_safety::SRSMode, functional_safety::SafetyTools)