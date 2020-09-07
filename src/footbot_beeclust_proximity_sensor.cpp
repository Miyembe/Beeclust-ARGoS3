
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/core/simulator/simulator.h>

#include "footbot_beeclust_proximity_sensor.h"

namespace argos {

 /****************************************/
 /****************************************/

 class CProximitySensorImpl : public CBeeClustProximityDefaultSensor {

 public:

 virtual Real CalculateReading(Real f_distance) {
 if(f_distance < 0.009889556) {
 return 1.0;
 }
 else {
 return 0.0100527 / (f_distance + 0.000163144);
 }
 }

 };

 /****************************************/
 /****************************************/

 CFootBotBeeClustProximitySensor::CFootBotBeeClustProximitySensor() :
 m_pcProximityImpl(new CProximitySensorImpl()) {}

 /****************************************/
 /****************************************/

 CFootBotBeeClustProximitySensor::~CFootBotBeeClustProximitySensor() {
 delete m_pcProximityImpl;
 }

 /****************************************/
 /****************************************/

 void CFootBotBeeClustProximitySensor::SetRobot(CComposableEntity& c_entity) {
 try {
 m_pcProximityImpl->SetRobot(c_entity);
 }
 catch(CARGoSException& ex) {
 THROW_ARGOSEXCEPTION_NESTED("Can't set robot for the foot-bot proximity default sensor", ex);
 }
 }

 /****************************************/
 /****************************************/

 void CFootBotBeeClustProximitySensor::Init(TConfigurationNode& t_tree) {
 m_pcProximityImpl->Init(t_tree);
 }

 /****************************************/
 /****************************************/

 void CFootBotBeeClustProximitySensor::Update() {
 m_pcProximityImpl->Update();
 for(size_t i = 0; i < m_pcProximityImpl->GetReadings().size(); ++i) {
 m_tReadings[i].Value = m_pcProximityImpl->GetReadings()[i];
 }
 }

 /****************************************/
 /****************************************/

 void CFootBotBeeClustProximitySensor::Reset() {
 m_pcProximityImpl->Reset();
 }

 /****************************************/
 /****************************************/

 REGISTER_SENSOR(CFootBotBeeClustProximitySensor,
 "footbot_beeclust_proximity", "default",
 "Seongin Na [respct11@gmail.com]",
 "1.0",
 "The foot-bot proximity sensor.",
 "This sensor accesses the foot-bot proximity sensor. For a complete description\n"
 "of its usage, refer to the ci_footbot_proximity_sensor.h interface. For the XML\n"
 "configuration, refer to the default proximity sensor.\n",
 "Usable"
 );

}