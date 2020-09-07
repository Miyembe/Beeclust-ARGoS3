
#ifndef QTOPENGL_FOOTBOT_BEECLUST_H
#define QTOPENGL_FOOTBOT_BEECLUST_H

namespace argos {
 class CQTOpenGLFootBotBeeClust;
 class CFootBotBeeClustEntity;
}

#ifdef __APPLE__
#include <gl.h>
#else
#include <GL/gl.h>
#endif

namespace argos {

 class CQTOpenGLFootBotBeeClust {

 public:

 CQTOpenGLFootBotBeeClust();

 virtual ~CQTOpenGLFootBotBeeClust();

 virtual void Draw(CFootBotBeeClustEntity& c_entity);

 protected:

 void MakeWheel();

 void SetWhitePlasticMaterial();
 void SetBlackTireMaterial();
 void SetCircuitBoardMaterial();
 void SetLEDMaterial(GLfloat f_red, GLfloat f_green, GLfloat f_blue);

 void RenderWheel();
 void RenderTrack();
 void RenderBase();
 void RenderGrippableSlice();
 void RenderGripperMechanics();
 void RenderGripperClaw();
 void RenderRAB();
 void RenderDistanceScannerSensor();
 void RenderDistanceScanner();
 void RenderIMX();
 void RenderBeacon();
 void RenderCamera();

 private:

 GLuint m_unLists;

 GLuint m_unBasicWheelList;

 GLuint m_unWheelList;
 GLuint m_unTrackList;
 GLuint m_unBaseList;
 GLuint m_unGrippableSliceList;
 GLuint m_unGripperMechanicsList;
 GLuint m_unGripperClawList;
 GLuint m_unRABList;
 GLuint m_unDistanceScannerSensorList;
 GLuint m_unDistanceScannerList;
 GLuint m_unIMXList;
 GLuint m_unBeaconList;
 GLuint m_unCameraList;

 GLuint m_unVertices;

 /* Angle gap between two leds */
 GLfloat m_fLEDAngleSlice;

 };

}

#endif