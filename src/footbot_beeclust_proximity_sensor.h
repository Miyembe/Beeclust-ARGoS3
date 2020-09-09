
#ifndef FOOTBOT_BEECLUST_PROXIMITY_SENSOR_H
#define FOOTBOT_BEECLUST_PROXIMITY_SENSOR_H

#include <string>
#include <map>

namespace argos {
 class CFootBotBeeClustProximitySensor;
 class CBeeClustProximitySensorEquippedEntity;
}

//#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include "ci_footbot_beeclust_proximity_sensor.h"
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/sensor.h>
//#include <argos3/plugins/robots/generic/simulator/proximity_default_sensor.h>
//#include "beeclust_proximity_default_sensor.h"
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

 virtual Real CalculateReading(Real f_distance);

 virtual Real CalculateNeighborDistance(CEmbodiedEntity* embodied_entity, SEmbodiedEntityIntersectionItem& interstected_robot);

 protected:

 /** Reference to embodied entity associated to this sensor */
 CEmbodiedEntity* m_pcEmbodiedEntity; 
 /** Reference to proximity sensor equipped entity associated to this sensor */
 CBeeClustProximitySensorEquippedEntity* m_pcProximityEntity; 
 /** Reference to controllable entity associated to this sensor */
 CControllableEntity* m_pcControllableEntity; 
 /** Flag to show rays in the simulator */
 bool m_bShowRays; 
 /** Random number generator */
 CRandom::CRNG* m_pcRNG; 
 /** Whether to add noise or not */
 bool m_bAddNoise; 
 /** Noise range */
 CRange<Real> m_cNoiseRange; 
 /** Reference to the space */
 CSpace& m_cSpace;
 };

}

#endif