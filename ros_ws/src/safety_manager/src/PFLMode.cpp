#include "safety_manager/SafetyTools.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twistwithcovariance.hpp"
#include "std_msgs/msg/bool.hpp"


namespace functional_safety
{
  class PFLMode : public SafetyTools
  {
  public:

      void initialize(const rclcpp::Node::SharedPtr & node) override
      {
          node_ = node;
          velocity_feedback_ = node_->create_subscription<geometry_msgs::msg::TwistWithCovariance>(
            "/velocity_feedback",10,std::bind(&PFLMode::checkVelocityLimits,this,std::placeholders _1)
          )
          emergency_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
            "/velocity_feedback",10)
          RCLCPP_INFO(node_->get_logger(), "[PFL] Initialized PFL safety mode.");
      }

      void stop() override
      {
        std_msgs::msg::Bool msg;
        msg.data = True;
        emergency_pub_.publish(msg);
      }

      void pause() override
      {
          return;
      }

      void resume() override
      {
        return;
      }

      void shutdown() override
      {

        if(velocity_feedback)
        {
            velocity_feedback.reset();
            RCLCPP_INFO(node_->get_logger(), "[PFL] Velocity feedback subscription shut down.");
        }
        if(emergency_pub_){
            emergency_pub_.reset();
            RCLCPP_INFO(node_->get_logger(),"[PFL] Emergency publisher shut down.");
        }
        RCLCPP_WARN(node_->get_logger(), "[PFL] Shutdown completed.");
      }

      void checkVelocityLimits(const std_msgs::msg::Float64 msg) override
      {
          //estrae le componenti di velocità
          double vx = msg->twist.twist.linear.x;
          double vy = msg->twist.twist.linear.y;
          double vz = msg->twist.twist.linear.z;
          // calcola il modulo
          double v = std::sqrt(vx*vx + vy*vy + vz*vz);
          //calcola la deviazione standard sul modulo
          double sigma_v = std::sqrt((vx*vx*sigma_x2 + vy*vy*sigma_y2 + vz*vz*sigma_z2)/(v*v));
          double k = 2.0; // livello di confidenza
          double safe_limit = vmax - k * sigma_v;
          if(v > safe_limit){
            stop();
            alertOperator();
          }
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
          RCLCPP_WARN(this->get_logger(), "Velocità modulare troppo alta! Fermare il robot");
          return;
      }

      void getFaultHistory() override
      {
          return;
      }

  private:
      rclcpp::Node::SharedPtr node_;
      rclcpp::Subscriber<geometry_msgs::msg::TwistWithCovariance>::SharedPtr velocity_feedback_;
      rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_pub_;
  }
};// namespace functional_safety

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(functional_safety::PFLMode, functional_safety::SafetyTools)