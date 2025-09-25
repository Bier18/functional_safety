#ifndef FUNCTIONAL_SAFETY_HPP
#define FUNCTIONAL_SAFETY_HPP
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "safety_manager/safety_states.hpp"


namespace functional_safety
{
  class SafetyTools
  {
    public:
      virtual void initialize(const rclcpp::Node::SharedPtr & node) = 0; // inizializza l'oggetto
      virtual void stop() = 0; //Arresto immediato del robot in caso di emergenza
      virtual void pause() = 0; //Fermata controllata, il robot è pronto a riprendere
      virtual void resume() = 0; //Riprende il movimento dopo la pausa
      virtual void shutdown() = 0; //Rimozione dei canali di comunicazione specifici della classe
      virtual void checkVelocityLimits() = 0; //controlla che le velocità dei giunti rimangano nei range
      virtual void checkTorqueLimits() = 0; //controlla che le coppie dei giunti rimangano nei range
      virtual void monitorSensors() = 0; //Monitora l’integrità dei sensori
      virtual void diagnose() = 0; //restituisce un report sulla salute del sistema
      virtual void safeMoveTo() = 0; //imposta un setpoint di posiione
      virtual void trajectoryCheck() = 0; // controlla che la traiettoria non ecceda i limiti di sicurezza
      virtual void ovverideControl() = 0; //permette il passaggio in modalità manuale
      virtual void logEvent() = 0; //registra anomalie
      virtual void alertOperator() = 0; //warning all'operatore
      virtual void getFaultHistory() = 0; //lista dei fault rilevati nel tempo
      virtual ~SafetyTools(){}

    protected:
      SafetyTools(){}
  };
}  // namespace functional_safety

#endif  // FUNCTIONAL_SAFETY_HPP