
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/core/simulator/simulator.h>
#include "beeclust_proximity_sensor_equipped_entity.h"
#include "footbot_beeclust_proximity_sensor.h"
#include <iostream>

namespace argos {

 static const Real BODY_RADIUS = 0.085036758f;
 static CRange<Real> UNIT(0.0f, 1.0f);

 /****************************************/
 /****************************************/


Real CFootBotBeeClustProximitySensor::CalculateReading(Real f_distance) {
//  if(f_distance < 0.009889556) {
//  return 1.0;
//  }
//  else {
 return 0.0100527 / (f_distance + 0.000163144);
//  }
 };

/* Calculate distance between the headings of the robot on which this sensor is attached and the robot detected by the center sensor. */
Real CFootBotBeeClustProximitySensor::CalculateNeighborDistance(CEmbodiedEntity* embodied_entity, 
                                                                SEmbodiedEntityIntersectionItem& intersected_robot) {
    CVector3 this_position;
    CVector3 intersected_position;
    this_position = embodied_entity->GetOriginAnchor().Position;
    intersected_position = intersected_robot.IntersectedEntity->GetOriginAnchor().Position;

    CQuaternion this_angle;
    CQuaternion intersected_angle;
    this_angle = embodied_entity->GetOriginAnchor().Orientation;
    intersected_angle = intersected_robot.IntersectedEntity->GetOriginAnchor().Orientation;
    
    CVector3 cHeading1;
    CVector3 cHeading2;

    cHeading1.Set(BODY_RADIUS, 0.0f, 0.0f);
    cHeading1.Rotate(this_angle);
    cHeading1 += this_position;

    cHeading2.Set(BODY_RADIUS, 0.0f, 0.0f);
    cHeading2.Rotate(intersected_angle);
    cHeading2 += intersected_position;

    Real neighbor_distance;

    neighbor_distance = Distance(cHeading1, cHeading2);

    return neighbor_distance;
}

 /****************************************/
 /****************************************/

 CFootBotBeeClustProximitySensor::CFootBotBeeClustProximitySensor() :
 m_pcEmbodiedEntity(NULL),
 m_bShowRays(false),
 m_pcRNG(NULL),
 m_bAddNoise(false),
 m_cSpace(CSimulator::GetInstance().GetSpace()) {}

 /****************************************/
 /****************************************/

 CFootBotBeeClustProximitySensor::~CFootBotBeeClustProximitySensor() {
 
 }

 /****************************************/
 /****************************************/

 void CFootBotBeeClustProximitySensor::SetRobot(CComposableEntity& c_entity) {
 try {
 m_pcEmbodiedEntity = &(c_entity.GetComponent<CEmbodiedEntity>("body"));
 m_pcControllableEntity = &(c_entity.GetComponent<CControllableEntity>("controller"));
 m_pcProximityEntity = &(c_entity.GetComponent<CBeeClustProximitySensorEquippedEntity>("beeclustproximity_sensors"));
 m_pcProximityEntity->Enable();
 }
 catch(CARGoSException& ex) {
 THROW_ARGOSEXCEPTION_NESTED("babo Can't set robot for the proximity sensor", ex);
 }
 }

 /****************************************/
 /****************************************/

 void CFootBotBeeClustProximitySensor::Init(TConfigurationNode& t_tree) {
 try {
 CCI_FootBotBeeClustProximitySensor::Init(t_tree);
 /* Show rays? */
 GetNodeAttributeOrDefault(t_tree, "show_rays", m_bShowRays, m_bShowRays);
 /* Parse noise level */
 Real fNoiseLevel = 0.0f;
 GetNodeAttributeOrDefault(t_tree, "noise_level", fNoiseLevel, fNoiseLevel);
 if(fNoiseLevel < 0.0f) {
 THROW_ARGOSEXCEPTION("Can't specify a negative value for the noise level of the proximity sensor");
 }
 else if(fNoiseLevel > 0.0f) {
 m_bAddNoise = true;
 m_cNoiseRange.Set(-fNoiseLevel, fNoiseLevel);
 m_pcRNG = CRandom::CreateRNG("argos");
 }
 m_tReadings.resize(m_pcProximityEntity->GetNumSensors());
 }
 catch(CARGoSException& ex) {
 THROW_ARGOSEXCEPTION_NESTED("Initialization error in proximity sensor", ex);
 }
 }

 /****************************************/
 /****************************************/

 void CFootBotBeeClustProximitySensor::Update() {
 /* Ray used for scanning the environment for obstacles */
 CRay3 cScanningRay;
 CVector3 cRayStart, cRayEnd;
 /* Buffers to contain data about the intersection */
 SEmbodiedEntityIntersectionItem sIntersection;
 //static CSimulator& cSimulator = CSimulator::GetInstance();
 /* Go through the sensors */
 for(UInt32 i = 0; i < m_tReadings.size(); ++i) {
    /* Compute ray for sensor i */
    cRayStart = m_pcProximityEntity->GetSensor(i).Offset;
    cRayStart.Rotate(m_pcProximityEntity->GetSensor(i).Anchor.Orientation);
    cRayStart += m_pcProximityEntity->GetSensor(i).Anchor.Position;
    cRayEnd = m_pcProximityEntity->GetSensor(i).Offset;
    cRayEnd += m_pcProximityEntity->GetSensor(i).Direction;
    cRayEnd.Rotate(m_pcProximityEntity->GetSensor(i).Anchor.Orientation);
    cRayEnd += m_pcProximityEntity->GetSensor(i).Anchor.Position;
    cScanningRay.Set(cRayStart,cRayEnd);
    /* Compute reading */
    /* Get the closest intersection */
    if(GetClosestEmbodiedEntityIntersectedByRay(sIntersection,
        cScanningRay,
        *m_pcEmbodiedEntity)) {
    /* There is an intersection */
        if(m_bShowRays) {
            m_pcControllableEntity->AddIntersectionPoint(cScanningRay,
            sIntersection.TOnRay);
            m_pcControllableEntity->AddCheckedRay(true, cScanningRay);
        }
        m_tReadings[i].Value = CalculateReading(cScanningRay.GetDistance(sIntersection.TOnRay));
        // std::cout << "The intersected entity with " << m_pcEmbodiedEntity->GetContext() << m_pcEmbodiedEntity->GetId() << "/ position: " << m_pcEmbodiedEntity->GetOriginAnchor().Position \
        //           << " is: " << sIntersection.IntersectedEntity->GetContext() << sIntersection.IntersectedEntity->GetId() << "/ position: " << sIntersection.IntersectedEntity->GetOriginAnchor().Position\
        //           << " Neighbor Distance : " << CalculateNeighborDistance(m_pcEmbodiedEntity, sIntersection)
        //           << std::endl;
        m_tNeighbors[i].Distance = CalculateNeighborDistance(m_pcEmbodiedEntity, sIntersection);
        m_tNeighbors[i].Intersect = true;
        // m_tDistances[i] = CalculateNeighborDistance(m_pcEmbodiedEntity, sIntersection);
        }
    else {
    /* No intersection */
        m_tReadings[i].Value = 0.0f;
        m_tNeighbors[i].Distance = 969.0f;
        m_tNeighbors[i].Intersect = false;
        if(m_bShowRays) {
            m_pcControllableEntity->AddCheckedRay(false, cScanningRay);
        }
    }
    /* Apply noise to the sensor */
    if(m_bAddNoise) {
        m_tReadings[i].Value += m_pcRNG->Uniform(m_cNoiseRange);
    }
    /* Trunc the reading between 0 and 1 */
    UNIT.TruncValue(m_tReadings[i].Value);
 }
 }

 /****************************************/
 /****************************************/

 void CFootBotBeeClustProximitySensor::Reset() {
 for(UInt32 i = 0; i < GetReadings().size(); ++i) {
 m_tReadings[i].Value = 0.0f;
 }
 for(UInt32 i = 0; i < GetNeighbors().size(); ++i) {
 m_tNeighbors[i].Distance = 969.0f;
 m_tNeighbors[i].Intersect = false;
 }
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