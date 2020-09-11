/* Include the controller definition */
#include "footbot_beeclust.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
#include <iostream>
#include "ctimer.h"

/****************************************/
static const Real BODY_RADIUS = 0.085036758f;
static const Real MAX_WAITING = 20000000;

/****************************************/
/****************************************/

CFootBotBeeClust::CFootBotBeeClust() :
   m_pcWheels(NULL),
   m_pcProximity(NULL),
   m_cAlpha(10.0f),
   m_fDelta(0.5f),
   m_fWheelVelocity(2.5f),
   m_fDriftVelocity(25.0f),
   m_fLeftPheroVelocity(0.0f),
   m_fRightPheroVelocity(0.0f),
   m_bias(0.0f),
   m_sensitivity(2.0f),
   m_isNeighborClose(false),
   m_cGoStraightAngleRange(-ToRadians(m_cAlpha),
                           ToRadians(m_cAlpha)) {}

/****************************************/
/****************************************/

void CFootBotBeeClust::Init(TConfigurationNode& t_node) {
   /*
    * Get sensor/actuator handles
    *
    * The passed string (ex. "differential_steering") corresponds to the
    * XML tag of the device whose handle we want to have. For a list of
    * allowed values, type at the command prompt:
    *
    * $ argos3 -q actuators
    *
    * to have a list of all the possible actuators, or
    *
    * $ argos3 -q sensors
    *
    * to have a list of all the possible sensors.
    *
    * NOTE: ARGoS creates and initializes actuators and sensors
    * internally, on the basis of the lists provided the configuration
    * file at the <controllers><footbot_diffusion><actuators> and
    * <controllers><footbot_diffusion><sensors> sections. If you forgot to
    * list a device in the XML and then you request it here, an error
    * occurs.
    */
   m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcProximity = GetSensor  <CCI_FootBotBeeClustProximitySensor>("footbot_beeclust_proximity");
   /*
    * Parse the configuration file
    *
    * The user defines this part. Here, the algorithm accepts three
    * parameters and it's nice to put them in the config file so we don't
    * have to recompile if we want to try other settings.
    */
   GetNodeAttributeOrDefault(t_node, "alpha", m_cAlpha, m_cAlpha);
   m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
   GetNodeAttributeOrDefault(t_node, "delta", m_fDelta, m_fDelta);
   GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);
}

/****************************************/
/****************************************/

void CFootBotBeeClust::ControlStep() {
   /* Get readings from proximity sensor */
   const CCI_FootBotBeeClustProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
   const CCI_FootBotBeeClustProximitySensor::TNeighbors& tNeighbors = m_pcProximity->GetNeighbors();
   /* Sum them together */
   CVector2 cAccumulator;
   /* Get proximity & neighbor distance & status if the neighbor is close */
   for(size_t i = 0; i < tProxReads.size(); ++i) {
      cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
      std::cout << "Distance Reading : " << tNeighbors[i].Distance << std::endl;
      if (tNeighbors[i].Distance <= BODY_RADIUS) {
         m_isNeighborClose = true;
      }
   }
   cAccumulator /= tProxReads.size();

   /* Set the wheel speed & waiting time based on the pheromone value */
   m_sensitivity = 0.2f;
   m_bias = 10.0f - (m_pValue[0] + m_pValue[1])*4;
   m_fLeftPheroVelocity = (m_pValue[1] - m_pValue[0])/m_sensitivity + m_bias;
   m_fRightPheroVelocity = (m_pValue[0] - m_pValue[1])/m_sensitivity + m_bias;
   m_waitingtime = CalculateWaiting(MAX_WAITING, (m_pValue[0] + m_pValue[1])/2);
   /* If the angle of the vector is small enough and the closest obstacle
    * is far enough, continue going straight, otherwise curve a little
    */
   CRadians cAngle = cAccumulator.Angle();
   if(m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) &&
      cAccumulator.Length() < m_fDelta ) {
      /* Go straight */
      m_pcWheels->SetLinearVelocity(m_fLeftPheroVelocity, m_fRightPheroVelocity);
   }
   else {
      /* Turn, depending on the sign of the angle */
      if(cAngle.GetValue() > 0.0f) {
         m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fDriftVelocity);
      }
      else {
         m_pcWheels->SetLinearVelocity(m_fDriftVelocity, m_fWheelVelocity);
      }
   }
   /* Wait timer on when it neighbor is close enough */
   if (m_isNeighborClose == true && m_waitTimer.paused()) {
      m_waitTimer.start();
   }

   /* End escape timer */
   if (m_escTimer.getTime() > 10000000) {
      m_escTimer.reset();
   }
   
   /* Waiting for 20s */ 
   if (!m_waitTimer.paused() && m_waitTimer.getTime() < m_waitingtime && m_escTimer.paused()) {
       m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
   }
   else if (!m_waitTimer.paused() && m_waitTimer.getTime() >= m_waitingtime) {
      m_waitTimer.reset();
      m_isNeighborClose = false;
      m_escTimer.start();
   }
   else {
      m_isNeighborClose = false;
   }
   std::cout << "Pheromone Value: " << m_pValue[0] << ", " << m_pValue[1] << std::endl;


   

//    /* Initialisation of wheel speeds */
//    m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
//    /* Get readings from proximity sensor */
//    const CCI_FootBotBeeClustProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
//    Real cAccumulator;
//    /* Read the 3 sensor values */
//    for(size_t i = 0; i < tProxReads.size(); ++i) {
//       sensor_val[i] = tProxReads[i].Value;
//       cAccumulator += tProxReads[i].Value;
//    }
//    cAccumulator /= tProxReads.size();
//    Real sensor_r = sensor_val[0];
//    Real sensor_c = sensor_val[1];
//    Real sensor_l = sensor_val[2];
//    if (cAccumulator >= 0.5) {
//       if (sensor_r > sensor_l) {
//          m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fDriftVelocity);
//       }
//       else {
//          m_pcWheels->SetLinearVelocity(m_fDriftVelocity, m_fWheelVelocity);
//       }
//    }
//    else {
//       /* go straight */
//       m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
//    }

}

Real* CFootBotBeeClust::GetPhero() {
   return m_pValue;
}

void CFootBotBeeClust::SetPhero(Real* pValue) {
   m_pValue[0] = pValue[0];
   m_pValue[1] = pValue[1];
}

int CFootBotBeeClust::CalculateWaiting(int maxWaitingTime, Real avgPhero) {
   int waitingTime;
   waitingTime = (int) (maxWaitingTime * avgPhero);
   return waitingTime;
}
/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CFootBotBeeClust, "footbot_beeclust_controller")
