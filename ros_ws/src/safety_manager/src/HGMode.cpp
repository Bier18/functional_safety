#include "safety_manager/SafetyTools.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "safety_msgs/msg/positions.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <cmath>

namespace functional_safety{
    class HGMode : public functional_safety::SafetyTools{
        public:
            void initialize(const rclcpp::Node::SharedPtr &node){
                node_ = node;
                stop_signal_srv_ = node_->create_client<std_srvs::srv::SetBool>("/stop_robot");
                override_control_srv_ = node_->create_client<std_srvs::srv::SetBool>("/override_control");
                poses_sub_ = node_->create_subscription<safety_msgs::msg::Positions>("/tcp_op_pose",10,
                                std::bind(&HGMode::hg_manager,this,std::placeholders::_1));
                manual_control_ = false;
                min_distance_ = 0.0;
                stop_= false;
                RCLCPP_INFO(node_->get_logger(),"[HG] Succesfully Enabled");
            }

            void stop() override {
                stop_ = true;
                RCLCPP_WARN(node_->get_logger(), "[HG] Stopping robot");
                //crea il messaggio di stop e lo pubblica
                auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
                request->data = true;
                // gestisce la risposta
                auto result = stop_signal_srv_->async_send_request(request,
                    [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture result){
                    if(result.valid()){
                        RCLCPP_INFO(node_->get_logger(), "%s", result.get()->message.c_str());
                    }
                    else{
                        RCLCPP_ERROR(node_->get_logger(), "Failed to call service");
                    }
                });
            } 

            void pause() override {return;} 

            void resume() override {
                stop_ = false;
                RCLCPP_WARN(node_->get_logger(), "[HG] Moving robot");
                //crea il messaggio di stop e lo pubblica
                auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
                request->data = false;
                // gestisce la risposta
                auto result = stop_signal_srv_->async_send_request(request,
                    [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture result){
                    if(result.valid()){
                        RCLCPP_INFO(node_->get_logger(), "%s", result.get()->message.c_str());
                    }
                    else{
                        RCLCPP_ERROR(node_->get_logger(), "Failed to call service");
                    }
                });
            } 

            void shutdown() override {
                manual_control_ = false;

                if(stop_signal_srv_)
                {
                    stop_signal_srv_.reset();
                    RCLCPP_INFO(node_->get_logger(), "[HG] Stop signal server shut down.");
                }
                if(override_control_srv_){
                    override_control_srv_.reset();
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
                //crea il messaggio del cambio di controllo
                auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
                request->data = manual_control_;
                // gestisce la risposta
                auto result = override_control_srv_->async_send_request(request,
                    [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture result){
                    if(result.valid()){
                        RCLCPP_INFO(node_->get_logger(), "%s", result.get()->message.c_str());
                    }
                    else{
                        RCLCPP_ERROR(node_->get_logger(), "Failed to call service");
                    }
                });
            }
            
            void logEvent() override {return;}

            void alertOperator() override {return;}

            void getFaultHistory() override {return;}

            void set_safety_params(double param) override {min_distance_ = param;}
        
        private:
            rclcpp::Node::SharedPtr node_;
            rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr stop_signal_srv_;
            rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr override_control_srv_;
            rclcpp::Subscription<safety_msgs::msg::Positions>::SharedPtr poses_sub_;
            bool manual_control_;
            bool stop_;
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
                else if(stop_){
                    resume();
                }
            }
    };
}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(functional_safety::HGMode, functional_safety::SafetyTools)
