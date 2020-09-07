
#ifndef FOOTBOT_BEECLUST_PROXIMITY_SENSOR_H
#define FOOTBOT_BEECLUST_PROXIMITY_SENSOR_H

#include <string>
#include <map>

namespace argos {
 class CFootBotBeeClustProximitySensor;
}

//#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include "ci_footbot_beeclust_proximity_sensor.h"
//#include <argos3/plugins/robots/generic/simulator/proximity_default_sensor.h>
#include "beeclust_proximity_default_sensor.h"
namespace argos {

 class CFootBotBeeClustProximitySensor : public CCI_FootBotBeeClustProximitySensor,
 public CSimulatedSensor {

 public:

 CFootBotBeeClustProximitySensor();

 virtual ~CFootBotBeeClustProximitySensor();

 virtual void SetRobot(CComposableEntity& c_entity);

 virtual void Init(TConfigurationNode& t_tree);

 virtual void Update();

 virtual void Reset();

 private:

 CBeeClustProximityDefaultSensor* m_pcProximityImpl;

 };

}

#endif