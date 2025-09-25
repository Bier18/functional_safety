#include "rclcpp/rclcpp.h"
#include "safety_manager/safety_states.hpp"
#include "safety_manager/SafetyTools.hpp"
#include <pluginlib/class_loader.hpp>

class StateMachine : public rclcpp::Node
{
    public:
        StateMachine() : Node("State Machine"){
            current_state_ = functional_safety::SafetyState::SSM; //inizializza lo stato
            
            //crea il server per il passaggio a SRS
            srs_srv_ = this->create_service<safety_msgs::srv::SafetyRatedStop>("srs_request",10,
                       std::bind(&StateMachine::srs_request,this,std::placeholder::_1));
            
                       //crea il server per il passaggio a PFL
            pfl_srv_ = this->create_service<safety_msgs::srv::PowerForceLimiting>("pfl_request",10,
                       std::bind(&StateMachine::pfl_request,this,std::placeholder::_1));
        }
    private:
        rclcpp::Service<safety_msgs::srv::SafetyRatedStop>::SharedPtr srs_srv_; //servizio per il passaggio a SRS
        rclcpp::Service<safety_msgs::srv::PowerForceLimiting>::SharedPtr pfl_srv_; //server per il passaggio a PFL
        functional_safety::Safetystates current_state_; //memorizza lo stato corrente
        //collegamento con i plugin
        pluginlib::ClassLoader<functional_safety::SafetyTools> state_loader("functional_safety", 
                                                                            "functional_safety::SafetyTools");
        
        //callback per la gesitone della richiesta SRS
        void srs_request(const std::shared_ptr<safety_msgs::srv::SafetyRatedStop::Request> request,
                         std::shared_ptr<safety_msgs::srv::SafetyRatedStop::Response> response)
        {
            try{
                std::shared_ptr<functional_safety::SafetyTools> srs_obj = poly_loader.createSharedInstance("SRS");
            }
        }
}