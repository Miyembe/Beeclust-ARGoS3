
#ifndef CPHEROFIELD_H
#define CPHEROFIELD_H

#include <stdlib.h>
#include <stdio.h>
#include <string>
#include "ctimer.h"
#include <math.h>
#include <iostream>
#include <cstring>
// #include <opencv2/opencv.hpp>
// #include <opencv2/core/core.hpp>

class CPheroField 
{
    public:
        CPheroField(float wi,float he, float res, const std::string& str_id); // 20200910 Pheromone field constructor. Environmental Effect will be added.
        ~CPheroField();

        /*recompute the pheromone field*/
        void recompute();

        /*inject a circle of pheromone*/
        void circle(float x, float y, float res, float num, float radius);

        /*inject pheromone to a given spot*/
        //void add(int x, int y,int id,int num,int radius);

        /*inject pheromone around every pixel on a line
        between the last and current injected position*/
        //void addTo(int x, int y,int id,int num,int radius = 25);


        /*read pheromone value at a given spot*/
        float get(float x, float y, float res);
    

        /*clear the entire pheromone field*/
        void clear();
    
    private:
        int m_width;
        int m_height;
        float m_resolution;
        int m_size;
        std::string m_pName;

        CTimer m_timer;
        float* m_pData;
};

#endif