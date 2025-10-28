#include "safety_manager/SafetyTools.hpp"
#include "rclcpp/rclcpp.hpp"
#include "safety_msgs/msg/positions.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <cmath>

namespace functional_safety
{
  class SRSMode : public functional_safety::SafetyTools
  {
  public:

      void initialize(const rclcpp::Node::SharedPtr & node) override
      {
          node_ = node;
          emergency_srv_ = node_->create_client<std_srvs::srv::SetBool>("/stop_robot");
          human_presence_sub_ = node->create_subscription<safety_msgs::msg::Positions>(
            "human_monitoring",10,std::bind(&SRSMode::humanDetectedCallback,this,std::placeholders::_1)
          );
          emergency_active_ = false;
          RCLCPP_INFO(node_->get_logger(), "[SRS] Initialized SRS safety mode.");
      }

      void stop() override
      {
        RCLCPP_WARN(node_->get_logger(), "[SRS] Emergency Stop activated!");
        emergency_active_ = true;
        //crea il messaggio di stop e lo pubblica
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = emergency_active_;
        // gestisce la risposta
        auto result = emergency_srv_->async_send_request(request,
            [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture result){
            if(result.valid()){
                RCLCPP_INFO(node_->get_logger(), "%s", result.get()->message.c_str());
            }
            else{
                RCLCPP_ERROR(node_->get_logger(), "Failed to call service");
            }
        });
      }

      void pause() override
      {
          return;
      }

      void resume() override
      {
        RCLCPP_WARN(node_->get_logger(), "[SRS] Emergency Stop deactivated!");
        emergency_active_ = false;
        //crea il messaggio di stop e lo pubblica
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = emergency_active_;
        // gestisce la risposta
        auto result = emergency_srv_->async_send_request(request,
            [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture result){
            if(result.valid()){
                RCLCPP_INFO(node_->get_logger(), "%s", result.get()->message.c_str());
            }
            else{
                RCLCPP_ERROR(node_->get_logger(), "Failed to call service");
            }
        });
      }

      void shutdown() override
      {
        emergency_active_ = false;

        if(human_presence_sub_)
        {
            human_presence_sub_.reset();
            RCLCPP_INFO(node_->get_logger(), "[SRS] Human presence subscription shut down.");
        }
        if(emergency_srv_){
            emergency_srv_.reset();
            RCLCPP_INFO(node_->get_logger(),"[SRS] Emergency server shut down.");
        }
        RCLCPP_WARN(node_->get_logger(), "[SRS] Shutdown completed.");
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
          RCLCPP_WARN(node_->get_logger(), "[SRS] Operator alert triggered!");
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
      rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr emergency_srv_;
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