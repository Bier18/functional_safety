#include <rclcpp/rclcpp.hpp>
#include "safety_manager/SafetyTools.hpp"
#include "std_msgs/msg/bool.hpp"
#include "safety_msgs/msg/ssm.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <cmath>
#include <algorithm>

namespace functional_safety
{
  class SSMMode : public functional_safety::SafetyTools
  {
    public:
      void initialize(const rclcpp::Node::SharedPtr & node){
        node_ = node;
        ssm_sub_ = node_->create_subscription<safety_msgs::msg::SSM>("/ssm",10,
                   std::bind(&SSMMode::spider_sense,this,std::placeholders::_1));
        stop_srv_ = node_->create_client<std_srvs::srv::SetBool>("/stop_robot");
        stop_ = false;
        min_distance = 0.5;
        RCLCPP_INFO(node_->get_logger(),"[SSM] Succesfully enabled");
      }
      void stop() override {
        stop_ = true;
        RCLCPP_WARN(node_->get_logger(), "[SSM] Emergency Stop activated!");
        //crea il messaggio di stop e lo pubblica
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = stop_;
        // gestisce la risposta
        auto result = stop_srv_->async_send_request(request,
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
        RCLCPP_WARN(node_->get_logger(), "[SRS] Emergency Stop activated!");
        stop_ = false;
        //crea il messaggio di stop e lo pubblica
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = stop_;
        // gestisce la risposta
        auto result = stop_srv_->async_send_request(request,
            [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture result){
            if(result.valid()){
                RCLCPP_INFO(node_->get_logger(), "%s", result.get()->message.c_str());
            }
            else{
                RCLCPP_ERROR(node_->get_logger(), "Failed to call service");
            }
        });
      }
      void shutdown() override {return;}
      void checkVelocityLimits(const std::shared_ptr<geometry_msgs::msg::TwistWithCovariance> /*msg*/) override {return;} 
      void checkTorqueLimits() override {return;}
      void monitorSensors() override {return;}
      void diagnose() override {return;}
      void safeMoveTo() override {return;}
      void trajectoryCheck() override {return;}
      void overrideControl() override {return;}
      void logEvent() override {return;}
      void alertOperator() override {return;}
      void getFaultHistory() override {return;}
      void set_safety_params(double /*param*/) override {return;}

    private:
      rclcpp::Node::SharedPtr node_;
      rclcpp::Subscription<safety_msgs::msg::SSM>::SharedPtr ssm_sub_;
      rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr stop_srv_;
      double min_distance;
      bool stop_;

      void spider_sense(const safety_msgs::msg::SSM::SharedPtr msg){
        double k = 2.0; // confidence interval
        double reaction_time = msg->reaction_time; // robot reaction time
        double stop_time = msg->stop_time; // robot stop time
        double robot_stop_speed = msg->robot_stop_speed; //speed needed to stop the robot since the stop signal is received
        //find robot pose
        double rx = msg->robot_state.robot_pose.pose.position.x;
        double ry = msg->robot_state.robot_pose.pose.position.y;
        double rz = msg->robot_state.robot_pose.pose.position.z;
        double srx = msg->robot_state.robot_pose.covariance[0];
        double sry = msg->robot_state.robot_pose.covariance[7];
        double srz = msg->robot_state.robot_pose.covariance[14];
        double r_pose = std::sqrt(rx*rx + ry*ry + rz*rz);
        double sigma_r = 0.0;
        if(r_pose > 1e-6){
          sigma_r = std::sqrt((rx*rx*srx + ry*ry*sry + rz*rz*srz)/(r_pose*r_pose));
        }
        r_pose = r_pose + k*sigma_r;
        //find operator pose
        double ox = msg->operator_state.operator_pose.pose.position.x;
        double oy = msg->operator_state.operator_pose.pose.position.y;
        double oz = msg->operator_state.operator_pose.pose.position.z;
        double sox = msg->operator_state.operator_pose.covariance[0];
        double soy = msg->operator_state.operator_pose.covariance[7];
        double soz = msg->operator_state.operator_pose.covariance[14];
        double o_pose = std::sqrt(ox*ox + oy*oy + oz*oz);
        double sigma_o = 0.0;
        if(o_pose > 1e-6){
          sigma_o = std::sqrt((ox*ox*sox + oy*oy*soy + oz*oz*soz)/(o_pose*o_pose));
        }
        o_pose = o_pose + k*sigma_o;
        double op_velocity = 0.0;
        //if we have information on operator velocity
        if(msg->use_operator_velocity){
          //find versor from operator to robot
          double ux = rx-ox;
          double uy = ry-oy;
          double uz = rz-oz;
          double u_module = std::sqrt(ux*ux + uy*uy + uz*uz);
          if(u_module > 1e-6){
            ux = ux / u_module;
            uy = uy / u_module;
            uz = uz / u_module;       
            //find operator velocity towards the robot
            double ovx = msg->operator_state.operator_velocity.twist.linear.x;
            double ovy = msg->operator_state.operator_velocity.twist.linear.y;
            double ovz = msg->operator_state.operator_velocity.twist.linear.z;
            double c00 = msg->operator_state.operator_velocity.covariance[0];
            double c11 = msg->operator_state.operator_velocity.covariance[7];
            double c22 = msg->operator_state.operator_velocity.covariance[14];
            double c01 = msg->operator_state.operator_velocity.covariance[1];
            double c02 = msg->operator_state.operator_velocity.covariance[2];
            double c12 = msg->operator_state.operator_velocity.covariance[8];
            double op_velocity = ux*ovx + uy*ovy + uz*ovz;
            double sop_velocity = ux*ux*c00 + uy*uy*c11 + uz*uz*c22 + 2*(ux*uy*c01 + ux*uz*c02 + uy*uz*c12);
            op_velocity = std::max(0.0,op_velocity + k*sop_velocity);
          }
        }else{
          op_velocity = 1.6; //according to the standard the standard
        }
        //if we have information on robot velocity
        double r_velocity = 0.0;
        if(msg->use_robot_velocity){
          //find versor from robot to operator
          double ux = ox-rx;
          double uy = oy-ry;
          double uz = oz-rz;
          double u_module = std::sqrt(ux*ux + uy*uy + uz*uz);
          if(u_module > 1e-6){
            ux = ux / u_module;
            uy = uy / u_module;
            uz = uz / u_module;       
            //find robot velocity towards the operato
            double rvx = msg->robot_state.robot_velocity.twist.linear.x;
            double rvy = msg->robot_state.robot_velocity.twist.linear.y;
            double rvz = msg->robot_state.robot_velocity.twist.linear.z;
            double c00 = msg->robot_state.robot_velocity.covariance[0];
            double c11 = msg->robot_state.robot_velocity.covariance[7];
            double c22 = msg->robot_state.robot_velocity.covariance[14];
            double c01 = msg->robot_state.robot_velocity.covariance[1];
            double c02 = msg->robot_state.robot_velocity.covariance[2];
            double c12 = msg->robot_state.robot_velocity.covariance[8];
            double r_velocity = ux*rvx + uy*rvy + uz*rvz;
            double sr_velocity = ux*ux*c00 + uy*uy*c11 + uz*uz*c22 + 2*(ux*uy*c01 + ux*uz*c02 + uy*uz*c12);
            r_velocity = std::max(0.0,r_velocity + k*sr_velocity);
          }
        }else{
          r_velocity = msg->tcp_to_op_max_speed; //according to the standard
        }
        //find minimum distance
        double Sh = op_velocity*(reaction_time + stop_time);
        double Sr = r_velocity*reaction_time;
        double Ss = robot_stop_speed*stop_time;
        double C = msg->safety_margin;
        min_distance = Sh+Sr+Ss+C;

        //if the robot and the operator are too close
        double rob_op_distance_x = rx-ox;
        double rob_op_distance_y = ry-oy;
        double rob_op_distance_z = rz-oz;
        double rob_op_distance = std::sqrt(rob_op_distance_x*rob_op_distance_x +
                                           rob_op_distance_y*rob_op_distance_y + 
                                           rob_op_distance_z*rob_op_distance_z);
        double rob_op_distance_sigma = sigma_o + sigma_r;
        rob_op_distance = rob_op_distance + k*rob_op_distance_sigma;
        if(rob_op_distance < min_distance && !stop_){
          stop();
        }else if(rob_op_distance > min_distance && stop_){
          resume();
        }
      }
  };
}  // namespace functional_safety

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(functional_safety::SSMMode, functional_safety::SafetyTools)