
#ifndef BEECLUST_PROXIMITY_SENSOR_EQUIPPED_ENTITY_H
#define BEECLUST_PROXIMITY_SENSOR_EQUIPPED_ENTITY_H

namespace argos {
 class CBeeClustProximitySensorEquippedEntity;
}

#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/simulator/entity/entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <map>

namespace argos {

 class CBeeClustProximitySensorEquippedEntity : public CEntity {

 public:

 ENABLE_VTABLE();

 struct SSensor {
 typedef std::vector<SSensor*> TList;

 CVector3 Offset;
 CVector3 Direction;
 SAnchor& Anchor;

 SSensor(const CVector3& c_offset,
 const CVector3& c_direction,
 Real f_range,
 SAnchor& s_anchor);
 };

 public:

 CBeeClustProximitySensorEquippedEntity(CComposableEntity* pc_parent);

 CBeeClustProximitySensorEquippedEntity(CComposableEntity* pc_parent,
 const std::string& str_id);

 virtual ~CBeeClustProximitySensorEquippedEntity();

 virtual void Init(TConfigurationNode& t_tree);

 virtual std::string GetTypeDescription() const {
 return "beeclustproximity_sensors";
 }

 virtual void Enable();

 virtual void Disable();

 inline size_t GetNumSensors() const {
 return m_tSensors.size();
 }

 inline SSensor& GetSensor(size_t un_idx) {
 return *m_tSensors[un_idx];
 }

 inline SSensor::TList& GetSensors() {
 return m_tSensors;
 }

 void AddSensor(const CVector3& c_offset,
 const CVector3& c_direction,
 Real f_range,
 SAnchor& s_anchor);

 void AddSensorRing(const CVector3& c_center,
 Real f_radius,
 const CRadians& c_start_angle,
 Real f_range,
 UInt32 un_num_sensors,
 SAnchor& s_anchor);

 void AddSensorCustomRing(const CVector3& c_center,
 Real f_radius,
 const CRadians& c_start_angle,
 const CRadians& c_spacing_angle,
 Real f_range,
 UInt32 un_num_sensors,
 SAnchor& s_anchor);

 void AddSensorFan(const CVector3& c_center,
 Real f_radius,
 const CRadians& c_start_angle,
 const CRadians& c_end_angle,
 Real f_range,
 UInt32 un_num_sensors,
 SAnchor& s_anchor);

 protected:

 SSensor::TList m_tSensors;

 };

}

#endif