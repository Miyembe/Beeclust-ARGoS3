add_library(beeclust SHARED
  ci_footbot_beeclust_proximity_sensor.h
  ci_footbot_beeclust_proximity_sensor.cpp
  footbot_beeclust.h
  footbot_beeclust.cpp
  beeclust_proximity_sensor_equipped_entity.cpp
  beeclust_proximity_sensor_equipped_entity.h
  footbot_beeclust_proximity_sensor.h
  footbot_beeclust_proximity_sensor.cpp
  qtopengl_footbot_beeclust.h
  qtopengl_footbot_beeclust.cpp
  footbot_beeclust_entity.cpp
  footbot_beeclust_entity.h
  dynamics2d_footbot_beeclust_model.cpp
  dynamics2d_footbot_beeclust_model.h
  ctimer.cpp
  ctimer.h
  CPherofield.cpp
  CPherofield.h
  beeclust_loop_function.cpp
  beeclust_loop_function.h
  #beeclust_proximity_default_sensor.h
  #beeclust_proximity_default_sensor.cpp
  #ci_beeclust_proximity_sensor.cpp
  #ci_beeclust_proximity_sensor.h
)
target_link_libraries(beeclust
  argos3core_simulator
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  ${LUA_LIBRARIES})
