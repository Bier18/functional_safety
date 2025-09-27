#include "safety_manager/SafetyTools.hpp"
#include "rclcpp/rclcpp.hpp"
#include "safety_msgs/msg/safetymonitordata.hpp"
#include "geometry_msgs/msg/point.hpp"


namespace functional_safety
{
  class SRSMode : public SafetyTools
  {
  public:

      void initialize(const rclcpp::Node::SharedPtr & node) override
      {
          node_ = node;
          emergency_pub_ = node_->create_publisher<std_msgs::msg::Bool>("emergency_msg",10);
          human_presence_sub_ = node->create_subscription<sensor_msgs::msg::SafetyMonitorData>(
            "human_monitoring",10,std::bind(&SRSMode::humanDetectedCallback,this,std::placeholders _1)
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

      void checkVelocityLimits() override
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

  private:
      rclcpp::Node::SharedPtr node_;
      bool emergency_active_;
      rclcpp::Subscription<safety_msgs::msg::SafetyMonitorData>::SharedPtr human_presence_sub_;
      rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_pub_;
      double min_distance_;

      void humanDetectedCallback(const safety_msgs::msg::SafetyMonitorData::SharedPtr msg)
      {
        if(!msg->valid || !msg->operator_state.valid){
            RCLCPP_WARN(node_->get_logger(), "[SRSMode] Not valid data! Triggering SRS stop.");
            stop();
            alertOperator();
            return;
        }

        geometry_msgs::Point op_pos = msg->operator_state.pose.pose.position;
        geometry_msgs::Point tcp_pos = msg->robot_state.tcp_pose.pose.position;
        double dx = op_pos.x - tcp_pos.x;
        double dy = op_pos.y - tcp_pos.y;
        double dz = op_pos.z - tcp_pos.z;
        double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
        if (!emergency_active_ && distance <= min_distance_)
        {
            RCLCPP_WARN(node_->get_logger(), "[SRSMode] Human detected! Triggering SRS stop.");
            stop();
        }else if (emergency_active_ && distance > min_distance){
            RCLCPP_WARN(node_->get_logger(), "[SRSMode] Human stepped away! Triggering SRS go.");
            resume();
        }
      }
  };

} // namespace functional_safety

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(functional_safety::SRSMode, functional_safety::SafetyTools)