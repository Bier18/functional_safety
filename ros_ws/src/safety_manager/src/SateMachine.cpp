#include "rclcpp/rclcpp.h"
#include "safety_manager/safety_states.hpp"
#include "safety_manager/SafetyTools.hpp"
#include <pluginlib/class_loader.hpp>
#include <limits>
#include <cmath>
#include "safety_msgs/srv/handguided.hpp"
#include "safety_msgs/srv/powerforcelimiting.hpp"
#include "safety_msgs/srv/safetyratedstop.hpp"

class StateMachine : public rclcpp::Node
{
    public:
        StateMachine() : Node("State Machine"){
            current_state_ = functional_safety::SafetyState::SSM; //inizializza lo stato

            obj = state_loader.createSharedInstance("SRS"); //carica il pugin di default
            obj->initialize(shared-from_this()); //inizializza il plugin
            
            //crea il server per il passaggio a SRS
            srs_srv_ = this->create_service<safety_msgs::srv::SafetyRatedStop>("srs_request",10,
                       std::bind(&StateMachine::srs_request,this,std::placeholder::_1));
            
            //crea il server per il passaggio a PFL
            pfl_srv_ = this->create_service<safety_msgs::srv::PowerForceLimiting>("pfl_request",10,
                       std::bind(&StateMachine::pfl_request,this,std::placeholder::_1));

            //crea il server per il passaggio a HG
            hg_srv_ = this->create_service<safety_msgs::srv::HandGuided>("hg_request",10,
                      std::bind(&StateMachine::hg_request,this,std::placeholder::_1));
        }
    private:
        rclcpp::Service<safety_msgs::srv::SafetyRatedStop>::SharedPtr srs_srv_; //servizio per il passaggio a SRS
        rclcpp::Service<safety_msgs::srv::PowerForceLimiting>::SharedPtr pfl_srv_; //server per il passaggio a PF
        rclcpp::Service<safety_msgs::srv::HandGuided>::SharedPtr hg_srv_; //server per il passaggio ad HG
        functional_safety::Safetystates current_state_; //memorizza lo stato corrente
        //collegamento con i plugin
        pluginlib::ClassLoader<functional_safety::SafetyTools> state_loader("functional_safety", 
                                                                            "functional_safety::SafetyTools");
        std::shared_ptr<functional_safety::SafetyTools> obj;
        
        //callback per la gesitone della richiesta SRS
        void srs_request(const std::shared_ptr<safety_msgs::srv::SafetyRatedStop::Request> request,
                         std::shared_ptr<safety_msgs::srv::SafetyRatedStop::Response> response)
        {
            if(request->srs_request){ // se richiedo l'attivazione
                try{
                    if(current_state_ != functional_safety::SafetyState::SRS){ // e non sono già in SRS
                        //entra in SRS
                        change_state("SRS");
                        response->srs_response = true; //accetta
                        response->message = "ok";
                    }else{ 
                        response->srs_response = false; //rifiuta
                        response->reason = "SRS mode already active"; //spiega il motivo del rifiuto
                    }
                }catch(pluginlib::PluginlibException& ex){
                    response->srs_response = false; //rifiuta
                    response->reason = ex.what();
                }
            }else{ //se richiedo la disattivazione
                try{
                    if(current_state_ == functional_safety::SafetyState::SRS){ // e sono in SRS
                        // ritorna allo stato di default
                        change_state("SSM");
                        response->srs_response = true; //accetta
                        response->reason = "ok";
                    }else{
                        response->srs_response = false; //rifiuta
                        response->reason = "SRS mode not active";
                    }
                }catch(pluginlib::PluginlibException& ex){
                    response->srs_response = false; //rifiuta
                    response->reason = ex.what();
                }
            }
        }

        //callback per la gestione della richiesta PFL
        void pfl_request(const std::shared_ptr<safety_msgs::srv::PowerForceLimiting::Request> request,
                         std::shared_ptr<safety_msgs::srv::PowerForceLimiting::Response> response)
        {
            if(request->pfl_request){ // se richiedo l'attivazione
                try{
                    if(current_state_ != functional_safety::SafetyState::PFL){ // e non sono già in PFL
                        //entra in PFL
                        change_state("PFL");
                        double vmax;
                        if(request->use_force){
                            force_max = request->force_max;
                            total_mass = request->total_mass;
                            k = request->k;
                            vmax = force_max/std::sqrt(total_mass*k);
                        }
                        else{
                            pressure_max = request->pressure_max;
                            contact_area = request->contact_area;
                            total_mass = request->total_mass;
                            k = request->k;
                            vmax = pressure_max*contact_area/std::sqrt(total_mass*k);
                        }
                        response->success = true; //accetta
                        response->message = "PFL mode active";
                        response->vmax = vmax;
                    }else{ 
                        response->success = false; //rifiuta
                        response->message = "PFL mode already active"; //spiega il motivo del rifiuto
                        response->vmax = std::numeric_limits<double>::quiet_NaN(""); //non ritorna un valore per la velocità
                    }
                }catch(pluginlib::PluginlibException& ex){
                    response->success = false; //rifiuta
                    response->message = ex.what();
                    response->vmax = std::numeric_limits<double>::quiet_NaN("");
                }
            }else{ //se richiedo la disattivazione
                try{
                    if(current_state_ == functional_safety::SafetyState::PFL){ // e sono in PFL
                        // ritorna allo stato di default
                        change_state("SSM");
                        response->success = true; //accetta
                        response->message = "PFL mode active";
                        response->vmax = std::numeric_limits<double>::quiet_NaN("");
                    }else{
                        response->success = false; //rifiuta
                        response->message = "PFL mode not active"; //spiega il motivo del rifiuto
                        response->vmax = std::numeric_limits<double>::quiet_NaN(""); //non ritorna un valore di velocità
                    }
                }catch(pluginlib::PluginlibException& ex){
                    response->success = false; //rifiuta
                    response->message = ex.what();
                    response->vmax = std::numeric_limits<double>::quiet_NaN("");
                }
            }
        }

        //callback per la gestione della richiesta HG
        void hg_request(const std::shared_ptr<safety_msgs::srv::HandGuided::Request> request,
                         std::shared_ptr<safety_msgs::srv::HandGuided::Response> response)
        {
            if(request->hg_request){ // se richiedo l'attivazione
                try{
                    if(current_state_ != functional_safety::SafetyState::HG){ // e non sono già in HG
                        //entra in HG
                        change_state("HG");
                        response->hg_response = true; //accetta
                        response->reason = "ok";
                    }else{ 
                        response->hg_response = false; //rifiuta
                        response->reason = "HG mode already active"; //spiega il motivo del rifiuto
                    }
                }catch(pluginlib::PluginlibException& ex){
                    response->hg_response = false; //rifiuta
                    response->reason = ex.what(); //spiega l'errore
                }
            }else{ //se richiedo la disattivazione
                try{
                    if(current_state_ == functional_safety::SafetyState::HG){ // e sono in HG
                        // ritorna allo stato di default
                        change_state("SSM");
                        response->hg_response = true; //accetta
                        response->reason = "ok";
                    }else{
                        response->hg_response = false; //rifiuta
                        response->reason = "HG mode not active"; //spiega il motivo del rifiuto
                    }
                }catch(pluginlib::PluginlibException& ex){
                    response->hg_response = false; //rifiuta
                    response->reason = ex.what(); //spiega l'errore
                }
            }
        }

        //implementa il cambio di stato
        void change_state(const std::string &state){
            obj->shutdown(); //resetta lo stato precedente precedente
            obj = state_loader.createSharedInstance(state); //carica il plugin
            obj->initialize(shared_from_this()); //inizializza il plugin
            //aggiorna current state
            if(state == "SSM"){
                current_state_ = functional_safety::SafetyState::SSM;
            }
            else if(state == "SRS"){
                current_state_ = functional_safety::SafetyState::SRS;
            }
            else if(state == "HG"){
                current_state_ = functional_safety::SafetyState::HG;
            }
            else if (state == "PFL"){
                current_state_ = functional_safety::SafetyState::PFL;
            }
            else{
                RCLCPP_WARN(this->get_logger(), "Unknown state requested: %s", state.c_str());
            }
        }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<StateMachine>());
    rclcpp::shutdown();
    return 0;
}