
#include "ci_beeclust_proximity_sensor.h"

#ifdef ARGOS_WITH_LUA
#include <argos3/core/wrappers/lua/lua_utility.h>
#endif

namespace argos {

 /****************************************/
 /****************************************/

 const std::vector<Real>& CCI_BeeClustProximitySensor::GetReadings() const {
 return m_tReadings;
 }

 /****************************************/
 /****************************************/

#ifdef ARGOS_WITH_LUA
 void CCI_BeeClustProximitySensor::CreateLuaState(lua_State* pt_lua_state) {
 CLuaUtility::OpenRobotStateTable(pt_lua_state, "beeclustproximity");
 for(size_t i = 0; i < m_tReadings.size(); ++i) {
 CLuaUtility::AddToTable(pt_lua_state, i+1, m_tReadings[i]);
 }
 CLuaUtility::CloseRobotStateTable(pt_lua_state);
 }
#endif

 /****************************************/
 /****************************************/

#ifdef ARGOS_WITH_LUA
 void CCI_BeeClustProximitySensor::ReadingsToLuaState(lua_State* pt_lua_state) {
 lua_getfield(pt_lua_state, -1, "beeclustproximity");
 for(size_t i = 0; i < m_tReadings.size(); ++i) {
 lua_pushnumber(pt_lua_state, i+1 );
 lua_pushnumber(pt_lua_state, m_tReadings[i]);
 lua_settable (pt_lua_state, -3 );
 }
 lua_pop(pt_lua_state, 1);
 }
#endif


 /****************************************/
 /****************************************/

}