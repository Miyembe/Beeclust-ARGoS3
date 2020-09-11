#include "CPherofield.h"

CPheroField::CPheroField(float wi, float he, float res, const std::string& str_id)
{
	// lastX = (int*)calloc(MAX_ID,sizeof(float));
	// lastY = (int*)calloc(MAX_ID,sizeof(float));
	m_width =  (int) wi / res;
	m_height = (int) he / res;
    m_resolution = res;
    
    m_pName = str_id;
	m_size = (int)m_width*m_height;
	m_pData = (float*)calloc(m_size,sizeof(float));
         
}


CPheroField::~CPheroField()
{
	free(m_pData);
}

void CPheroField::clear()
{
	memset(m_pData,0,m_size*sizeof(float));
}

void CPheroField::circle(float x, float y, float res, float num, float radius)
{
    int magX, magY, magR;
    magX = (int) (x / res);
    magY = (int) (y / res);
    magR = (int) (radius / res);
    int pos = 0;
    int iix,iiy;
    for (int ix = -magR;ix<magR+1;ix++){
		iix = ix + magX;
		for (int iy = -magR;iy<magR+1;iy++){
			iiy = iy + magY;
			if ((ix*ix)+(iy*iy)<=magR*magR){
				pos = (iix+iiy*m_width);
				m_pData[pos] += num;
			}
		}
	}
}

float CPheroField::get(float x, float y, float res)
{   int magX, magY;
    magX = (int) (x / res);
    magY = (int) (y / res);
	if (magX > 0 && magY >0 && magX<m_width && magY<m_height) return m_pData[magX+magY*m_width];
	//return -1;
    //return magX;
}

/* 20200910 To be applied - for evaporation */
// void CPheroField::recompute()
// {
// 	float timex = m_timer.getTime();
// 	float decay = pow(2,-timex/1000000.0/evaporation);
// 	//int diffV,diffH; // declaration of diffusion constants for each axe
// 	for (int i = 0;i<m_size;i++) m_pData[i]=m_pData[i]*decay;
	
// 	//printf("Recompute took %.0f %f\n",timer.getTime()-timex,diffuse);
// 	timer.reset();
// 	timer.start();
// }