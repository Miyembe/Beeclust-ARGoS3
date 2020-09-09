#ifndef CCI_FOOTBOT_BEECLUST_PROXIMITY_SENSOR_H
#define CCI_FOOTBOT_BEECLUST_PROXIMITY_SENSOR_H

namespace argos {
 class CCI_FootBotBeeClustProximitySensor;
}

#include <argos3/core/control_interface/ci_sensor.h>
#include <argos3/core/utility/math/angles.h>
#include <argos3/core/simulator/entity/embodied_entity.h>

namespace argos {

 class CCI_FootBotBeeClustProximitySensor : public CCI_Sensor {

 public:

 struct SReading {
 Real Value;
 CRadians Angle;

 SReading() :
 Value(0.0f) {}

 SReading(Real f_value,
 const CRadians& c_angle) :
 Value(f_value),
 Angle(c_angle) {}
 };

 struct SNeighbor {
 Real Distance;
 bool Intersect;
 
 SNeighbor() :
 Distance(0.0f), Intersect(false) {}

 SNeighbor(Real n_distance, bool isIntersect) :
 Distance(n_distance),
 Intersect(isIntersect) {}
 };

 typedef std::vector<SReading> TReadings;
 //typedef std::vector<Real> Reals;
 typedef std::vector<SNeighbor> TNeighbors;
 public:

 CCI_FootBotBeeClustProximitySensor();

 virtual ~CCI_FootBotBeeClustProximitySensor() {}

 const TReadings& GetReadings() const;
 //const Reals& GetDistances() const;
 const TNeighbors& GetNeighbors() const;

#ifdef ARGOS_WITH_LUA
 virtual void CreateLuaState(lua_State* pt_lua_state);

 virtual void ReadingsToLuaState(lua_State* pt_lua_state);
#endif

 protected:

 TReadings m_tReadings;
 //Reals m_tDistances;
 TNeighbors m_tNeighbors;

 };

 std::ostream& operator<<(std::ostream& c_os, const CCI_FootBotBeeClustProximitySensor::SReading& s_reading);
 std::ostream& operator<<(std::ostream& c_os, const CCI_FootBotBeeClustProximitySensor::TReadings& t_readings);

 std::ostream& operator<<(std::ostream& c_os, const CCI_FootBotBeeClustProximitySensor::SNeighbor& s_neighbor);
 std::ostream& operator<<(std::ostream& c_os, const CCI_FootBotBeeClustProximitySensor::TNeighbors& t_neighbors);
}

#endif