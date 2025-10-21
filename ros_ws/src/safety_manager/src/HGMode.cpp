#include "safety_manager/SafetyTools.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "safety_msgs/msg/positions.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include <cmath>

namespace functional_safety{
    class HGMode : public functional_safety::SafetyTools{
        public:
            void initialize(const rclcpp::Node::SharedPtr &node){
                node_ = node;
                stop_signal_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/hg_stop",10);
                override_control_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/override_control",10);
                poses_sub_ = node_->create_subscription<safety_msgs::msg::Positions>("/tcp_pose",10,
                                std::bind(&HGMode::hg_manager,this,std::placeholders::_1));
                manual_control_ = false;
                min_distance_ = 0.0;
                RCLCPP_INFO(node_->get_logger(),"[HG] Succesfully Enabled");
            }

            void stop() override {
                std_msgs::msg::Bool msg;
                msg.data = true;
                stop_signal_pub_->publish(msg);
                RCLCPP_WARN(node_->get_logger(),"[HG] Detected Human, stopping robot");
            } 

            void pause() override {return;} 

            void resume() override {
                std_msgs::msg::Bool msg;
                msg.data = false;
                stop_signal_pub_->publish(msg);
                RCLCPP_WARN(node_->get_logger(),"[HG] Resuming robot functionalities");
            } 

            void shutdown() override {
                manual_control_ = false;

                if(stop_signal_pub_)
                {
                    stop_signal_pub_.reset();
                    RCLCPP_INFO(node_->get_logger(), "[HG] Stop signal publisher shut down.");
                }
                if(override_control_pub_){
                    override_control_pub_.reset();
                    RCLCPP_INFO(node_->get_logger(),"[HG] Override control publisher shut down.");
                }
                RCLCPP_WARN(node_->get_logger(), "[HG] Shutdown completed.");
            } 

            void checkVelocityLimits(const std::shared_ptr<geometry_msgs::msg::TwistWithCovariance> /*msg*/) override {return;} 

            void checkTorqueLimits() override {return;} 

            void monitorSensors() override {return;}

            void diagnose() override {return;}

            void safeMoveTo() override {return;}

            void trajectoryCheck() override {return;}

            void overrideControl() override {
                stop();
                manual_control_ = !manual_control_;
                std_msgs::msg::Bool msg;
                msg.data = manual_control_;
                override_control_pub_->publish(msg);
            }
            
            void logEvent() override {return;}

            void alertOperator() override {return;}

            void getFaultHistory() override {return;}

            void set_safety_params(double param) override {min_distance_ = param;}
        
        private:
            rclcpp::Node::SharedPtr node_;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_signal_pub_;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr override_control_pub_;
            rclcpp::Subscription<safety_msgs::msg::Positions>::SharedPtr poses_sub_;
            bool manual_control_;
            double min_distance_;

            void hg_manager(const std::shared_ptr<safety_msgs::msg::Positions> msg){
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
                //find safe limit
                double k = 2.0; // confidence level
                double r_safe_limit = r_pose - k * sigma_r;
                double o_safe_limit = o_pose - k * sigma_o;
                //if the robot is in the collaborative workspace but the manaul control
                //is not enabled
                if(r_safe_limit < min_distance_ && !manual_control_){
                    //enable manual control
                    RCLCPP_INFO(node_->get_logger(),"[HG] Enabling manual override");
                    overrideControl();
                }
                //if the operator is moving away
                else if(o_safe_limit > min_distance_ && manual_control_){
                    //disable manual control
                    RCLCPP_INFO(node_->get_logger(),"[HG] Disabling manual override");
                    overrideControl();
                }
                else{
                    resume();
                }
            }
    };
}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(functional_safety::HGMode, functional_safety::SafetyTools)
