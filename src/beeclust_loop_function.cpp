#include <string.h>
#include "beeclust_loop_function.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include "footbot_beeclust_entity.h"
#include "footbot_beeclust.h"

/****************************************/
/****************************************/


      
//    m_cForagingArenaSideX(-0.9f, 1.7f),
//    m_cForagingArenaSideY(-1.7f, 1.7f),
//    m_pcFloor(NULL),
//    m_pcRNG(NULL),
//    m_unCollectedFood(0),
//    m_nEnergy(0),
//    m_unEnergyPerFoodItem(1),
//    m_unEnergyPerWalkingRobot(1) {
/****************************************/
/****************************************/

static const Real INTER_WHEEL_DISTANCE = 0.14f;
static const Real HALF_INTER_WHEEL_DISTANCE = INTER_WHEEL_DISTANCE * 0.5f;
CBeeClustLoopFunctions::CBeeClustLoopFunctions():
 m_resolution(0.0f),
 arena_width(0),
 arena_height(0)
       {}

/****************************************/
/****************************************/


/****************************************/
/****************************************/

void CBeeClustLoopFunctions::Init(TConfigurationNode& t_node) {

   try {
      TConfigurationNode& tPhero = GetNode(t_node, "pheromone_field");
      GetNodeAttribute(tPhero, "resolution", m_resolution);
   }

      


      /* Get the number of food items we want to be scattered from XML */
      //GetNodeAttribute(tForaging, "radius", m_fFoodSquareRadius);
      //m_fFoodSquareRadius *= m_fFoodSquareRadius;
      /* Create a new RNG */
      //m_pcRNG = CRandom::CreateRNG("argos");
      /* Distribute uniformly the items in the environment */
    //   for(UInt32 i = 0; i < unFoodItems; ++i) {
    //      m_cFoodPos.push_back(
    //         CVector2(m_pcRNG->Uniform(m_cForagingArenaSideX),
    //                  m_pcRNG->Uniform(m_cForagingArenaSideY)));
    //   }
      /* Get the output file name from XML */
      //GetNodeAttribute(tForaging, "output", m_strOutput);
      /* Open the file, erasing its contents */
      //m_cOutput.open(m_strOutput.c_str(), std::ios_base::trunc | std::ios_base::out);
      //m_cOutput << "# clock\twalking\tresting\tcollected_food\tenergy" << std::endl;
      /* Get energy gain per item collected */
      //GetNodeAttribute(tForaging, "energy_per_item", m_unEnergyPerFoodItem);
      /* Get energy loss per walking robot */
      //GetNodeAttribute(tForaging, "energy_per_walking_robot", m_unEnergyPerWalkingRobot);
   //}
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error parsing loop functions!", ex);
   }


   // 20200527 test
   // const UInt32 m_unPixelsPerMeter = 10;
   // CVector2 m_cHalfArenaSize;
   // CVector2 m_cArenaCenter;
   // CVector2 scaledArenaSize;

   arena_width = CSimulator::GetInstance().GetSpace().GetArenaSize().GetX();
   arena_height = CSimulator::GetInstance().GetSpace().GetArenaSize().GetY();
   const CVector3& cArenaCenter = CSimulator::GetInstance().GetSpace().GetArenaCenter();
   m_cArenaCenter.Set(cArenaCenter.GetX(), cArenaCenter.GetY());
   phero_field = new CPheroField(arena_width, arena_height, m_resolution, "test_pheromone");

   // set the circle radius as you want
   phero_field->circle(m_cArenaCenter.GetX()+ 0.5*arena_width, m_cArenaCenter.GetY()+ 0.5*arena_height, m_resolution, 1.0f, 1.0f);


   
 
 
}

/****************************************/
/****************************************/

void CBeeClustLoopFunctions::Reset() {

}

 

/****************************************/
/****************************************/

void CBeeClustLoopFunctions::Destroy() {
   /* Close the file */
   m_cOutput.close();
}

void CBeeClustLoopFunctions::PreStep() {

  CSpace::TMapPerType& m_cRobots = GetSpace().GetEntitiesByType("foot-bot-beeclust");
  for(CSpace::TMapPerType::iterator it = m_cRobots.begin();
       it != m_cRobots.end();
       ++it) 
  {
      CFootBotBeeClustEntity& cRobot = *any_cast<CFootBotBeeClustEntity*>(it->second);
      CFootBotBeeClust& cController = dynamic_cast<CFootBotBeeClust&>(cRobot.GetControllableEntity().GetController());
      CVector2 cPos;
      CQuaternion cOrientation;
      cPos.Set(cRobot.GetEmbodiedEntity().GetOriginAnchor().Position.GetX() + 0.5*arena_width,
              cRobot.GetEmbodiedEntity().GetOriginAnchor().Position.GetY() + 0.5*arena_height);
      cOrientation = cRobot.GetEmbodiedEntity().GetOriginAnchor().Orientation;
      CVector3 cWheelPos[2];
      cWheelPos[0].Set(-HALF_INTER_WHEEL_DISTANCE,
                       0.0f,
                       0.0f);
      cWheelPos[0].Rotate(cOrientation);
      cWheelPos[0] += cRobot.GetEmbodiedEntity().GetOriginAnchor().Position;
      cWheelPos[0] += CVector3(0.5*arena_width, 0.5*arena_height, 0.0f);
      cWheelPos[1].Set(-HALF_INTER_WHEEL_DISTANCE,
                       0.0f,
                       0.0f);
      cWheelPos[1].Rotate(cOrientation);
      cWheelPos[1] += cRobot.GetEmbodiedEntity().GetOriginAnchor().Position;
      cWheelPos[1] += CVector3(0.5*arena_width, 0.5*arena_height, 0.0f);

      Real cPhero[2];
      for (int i = 0; i < 2; i++) {
         cPhero[i] = phero_field->get(cWheelPos[i].GetX(), cWheelPos[i].GetY(), m_resolution);
      }
      cController.SetPhero(cPhero);
  }

}

/****************************************/
/****************************************/

void CBeeClustLoopFunctions::PostStep() {

}


// 20200529 Converting position value into the index for the pheromon matrix
// UInt32 CBeeClustLoopFunctions::PosToIdx(Real pos, UInt32 res, Real halfArena){
//    UInt32 idxForMat = static_cast<UInt32>(res * (pos + halfArena));
//    return idxForMat;
// }



/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CBeeClustLoopFunctions, "beeclust_loop_functions")

