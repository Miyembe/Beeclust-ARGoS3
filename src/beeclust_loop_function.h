#ifndef PHERO_LOOP_FUNCTIONS_H
#define PHERO_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>
#include "footbot_beeclust_entity.h"
#include "footbot_beeclust.h"
#include "CPherofield.h"

using namespace argos;

class CBeeClustLoopFunctions : public CLoopFunctions {

public:

   CBeeClustLoopFunctions();

   virtual ~CBeeClustLoopFunctions() {}

   virtual void Init(TConfigurationNode& t_tree);
   
   virtual void Reset();
   virtual void Destroy();

   virtual void PreStep();

   virtual void PostStep();

protected:


   


private:
   /*
   Real m_fFoodSquareRadius;
   CRange<Real> m_cForagingArenaSideX, m_cForagingArenaSideY;
   std::vector<CVector2> m_cFoodPos;
   CFloorEntity* m_pcFloor;
   CRandom::CRNG* m_pcRNG;

   std::string m_strOutput;
  

   UInt32 m_unCollectedFood;
   SInt64 m_nEnergy;
   UInt32 m_unEnergyPerFoodItem;
   UInt32 m_unEnergyPerWalkingRobot;*/
   std::ofstream m_cOutput;
   float m_resolution;
   CPheroField* phero_field;
   float arena_width;
   float arena_height;

   CVector2 m_cArenaCenter;


//    Real PheroStartX;
//    Real PheroStartY;
//    Real PheroEndX;
//    Real PheroEndY;

   


};

#endif
