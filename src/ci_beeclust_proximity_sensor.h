
#ifndef CCI_BEECLUST_PROXIMITY_SENSOR_H
#define CCI_BEECLUST_PROXIMITY_SENSOR_H

namespace argos {
 class CCI_BeeClustProximitySensor;
}

#include <argos3/core/control_interface/ci_sensor.h>

namespace argos {

 class CCI_BeeClustProximitySensor : public CCI_Sensor {

 public:

 virtual ~CCI_BeeClustProximitySensor() {}

 const std::vector<Real>& GetReadings() const;

#ifdef ARGOS_WITH_LUA
 virtual void CreateLuaState(lua_State* pt_lua_state);

 virtual void ReadingsToLuaState(lua_State* pt_lua_state);
#endif

 protected:

 std::vector<Real> m_tReadings;

 };

}

#endif