/* Header file Clhex_articulated_climb2 */
#ifndef __Clhex_articulated_climb2_HPP__
#define __Clhex_articulated_climb2_HPP__

/* interface */

#include"CtvSt.hpp"
#include"CtvDll.hpp"

EXTERN_C void cdecl UserCalc( VectRPtr _x, VectRPtr _v, VectRPtr _a, integer _isubs, integer _UMMessage, integer &WhatDo );
EXTERN_C void cdecl ControlPanelMessage( VectRPtr _x, VectRPtr _v, VectRPtr _a, integer _isubs, integer _index, double _Value );
EXTERN_C void cdecl TimeFuncCalc( real_ _t, VectRPtr _x, VectRPtr _v, integer _isubs );
EXTERN_C void cdecl Get1stOrderODE( real_ _t, VectRPtr _x, VectRPtr _f, integer _isubs );
EXTERN_C void cdecl Get2ndOrderODE( real_ _t, VectRPtr _x, VectRPtr _v, VectRPtr _f, integer _isubs );
EXTERN_C void cdecl UserConCalc( VectRPtr _x, VectRPtr _v, MatrRPtr _Jacobi, Vec3RPtr _Error, integer _isubs, integer _ic, boolean _predict, integer _nright );
/* implementation */

#include"DGetVars.hpp"
#include"hex_articulated_climb2C.hpp"
#include"_Thex_articulated_climb2.hpp"

void StepTrans(real_ eta, coordin &r, coordin &leg, coordin &shift, coordin &angle)
{

 trans_matr A_eta =
{{cos(eta),-sin(eta),0},
{sin(eta),cos(eta),0},
{0,0,1}};

trans_matr A_psi =
{{cos(angle[0]),-sin(angle[0]),0},
{sin(angle[0]),cos(angle[0]),0},
{0,0,1}};

trans_matr A_teta =
{{1,0,0},
{0,cos(angle[1]),-sin(angle[1])},
{0,sin(angle[1]),cos(angle[1])}};

trans_matr A_gamma =
{{cos(angle[2]),0,sin(angle[2])},
{0,1,0},
{-sin(angle[2]),0,cos(angle[2])}};



 Mult_m_v_3x1(NORMAL,A_eta,leg,leg);

 for(int i=0;i<3;i++)
 {
 leg[i] = leg[i]+r[i]-shift[i];
 }

Mult_m_v_3x1(TRANSPON,A_gamma,leg,leg);
Mult_m_v_3x1(TRANSPON,A_teta,leg,leg);
Mult_m_v_3x1(TRANSPON,A_psi,leg,leg);


for(int i=0;i<3;i++)
{
leg[i] = leg[i] - r[i];
}

 Mult_m_v_3x1(TRANSPON,A_eta,leg,leg);

}


void RevKinematics(coordin &leg, coordin &p,coordin &angle)
{
if(leg[2]<=0)
{
 angle[0] = atan2(leg[1],leg[0]);
 angle[1] = -arccos((leg[0]*cos(angle[0])+leg[1]*sin(angle[0])-p[0])/(pow(pow(leg[0]*cos(angle[0])+leg[1]*sin(angle[0])-p[0],2)+pow(leg[2],2),0.5)))+arccos((pow(p[1],2)+pow(leg[0]*cos(angle[0])+leg[1]*sin(angle[0])-p[0],2)+pow(leg[2],2)-pow(p[2],2))/(2*p[1]*pow(pow(leg[0]*cos(angle[0])+leg[1]*sin(angle[0])-p[0],2)+pow(leg[2],2),0.5)));
 angle[2] = -arccos((pow(leg[0]*cos(angle[0])+leg[1]*sin(angle[0])-p[0],2)+pow(leg[2],2)-pow(p[1],2)-pow(p[2],2))/(2*p[1]*p[2]));
}
else
{
 angle[0] = atan2(leg[1],leg[0]);
 angle[1] = +arccos((leg[0]*cos(angle[0])+leg[1]*sin(angle[0])-p[0])/(pow(pow(leg[0]*cos(angle[0])+leg[1]*sin(angle[0])-p[0],2)+pow(leg[2],2),0.5)))+arccos((pow(p[1],2)+pow(leg[0]*cos(angle[0])+leg[1]*sin(angle[0])-p[0],2)+pow(leg[2],2)-pow(p[2],2))/(2*p[1]*pow(pow(leg[0]*cos(angle[0])+leg[1]*sin(angle[0])-p[0],2)+pow(leg[2],2),0.5)));
 angle[2] = -arccos((pow(leg[0]*cos(angle[0])+leg[1]*sin(angle[0])-p[0],2)+pow(leg[2],2)-pow(p[1],2)-pow(p[2],2))/(2*p[1]*p[2]));
}
}

//функция поворота для первого и третьего сегментов
//angle1 = {delta,0,eta} - переход в СК ноги из СК№1
//ri - положение СК ноги во СК №1
//leg - координаты конца ноги в СК№1
//angle2 = {psi,teta,gamma} - углы поворота шагового цикла из-за поворота главного звена
//shift - сдвиг главного звена перед поворотом
//r1 - координаты шарнира delta в СК№2
//r2 - координаты шарнира delta  в СК№1
void StepTrans2(coordin &angle1, coordin &ri, coordin &leg, coordin &shift, coordin &angle2, coordin &r1, coordin &r2)
{

 trans_matr A_psi =
{{cos(angle2[0]),-sin(angle2[0]),0},
{sin(angle2[0]),cos(angle2[0]),0},
{0,0,1}};

trans_matr A_teta =
{{1,0,0},
{0,cos(angle2[1]),-sin(angle2[1])},
{0,sin(angle2[1]),cos(angle2[1])}};

trans_matr A_gamma =
{{cos(angle2[2]),0,sin(angle2[2])},
{0,1,0},
{-sin(angle2[2]),0,cos(angle2[2])}};

 trans_matr Ai =
 {{cos(angle1[2]),-sin(angle1[2]),0},
 {sin(angle1[2]),cos(angle1[2]),0},
 {0,0,1}};

 trans_matr A_delta =
 {{1,0,0},
 {0,cos(angle1[0]),-sin(angle1[0])},
 {0,sin(angle1[0]),cos(angle1[0])}};

 trans_matr A_PSI =
 {{cos(angle1[1]),-sin(angle1[1]),0},
 {sin(angle1[1]),cos(angle1[1]),0},
 {0,0,1}};

 Mult_m_v_3x1(NORMAL,Ai,leg,leg);

 for(int i=0;i<3;i++)
 {
   leg[i] = leg[i]+ri[i]+2*r1[i]-shift[i];
 }

 Mult_m_v_3x1(TRANSPON,A_gamma,leg,leg);
 Mult_m_v_3x1(TRANSPON,A_teta,leg,leg);
 Mult_m_v_3x1(TRANSPON,A_psi,leg,leg);

 for(int i=0;i<3;i++)
 {
 leg[i] = leg[i]-r1[i]-r2[i];
 }

 Mult_m_v_3x1(TRANSPON,A_delta,leg,leg);
 Mult_m_v_3x1(TRANSPON,A_PSI,leg,leg);

 for(int i=0;i<3;i++)
 {
 leg[i] = leg[i]-ri[i];
 }

 Mult_m_v_3x1(TRANSPON,Ai,leg,leg);
}

//простой шаговый цикл для движения по прямой
void Step1(integer index, real_ a, real_ b, real_ t, real_ W, coordin & leg)
{
	real_ local_time = t - (int)(t/(2*W))*(2*W);
	if (local_time<W)
	{
		if(index==1)
		{
		leg[0] = 0.4;
		leg[1] = a*cos((pi/W)*local_time);
		leg[2] = b*sin((pi/W)*local_time)-0.25;
		}
		else
		{
		leg[0] = 0.4;
		//leg[1] = a*cos((pi/W)*t);
		leg[1] = -a+2*(a/W)*local_time;
		leg[2] = -0.25;
		}
	}
	else if (local_time>W)
	{
		if(index==1)
		{
		leg[0] = 0.4;
		//leg[1] = a*cos((pi/W)*t);
		leg[1] = -a+2*(a/W)*(local_time-W);
		leg[2] = -0.25;		
		}
		else
		{
		leg[0] = 0.4;
		leg[1] = a*cos((pi/W)*(local_time-W));
		leg[2] = b*sin((pi/W)*(local_time-W))-0.25;		
		}
	}
}

/* Функция "TimeFuncCalc" используется исключительно для расчета функций времени.
Не используйте функции типа "GetPoint" внутри этой процедуры.
Недопустим расчет сил в этой процедуре. */
void TimeFuncCalc( real_ _t, VectRPtr _x, VectRPtr _v, integer _isubs )
 {
  _hex_articulated_climb2VarPtr _;
  _ = _PzAll[SubIndx[_isubs-1]-1];
 real_ w_body = _->w_body;
 real_ l_body = _->l_body;
 real_ p1 = _->p1;
 real_ p2 = _->p2;
 real_ p3 = _->p3;
 coordin p = {p1,p2,p3};
 real_ a = _->a;
 real_ b = _->b;
 real_ W = _->W;

 real_ delta1 = _->delta1;
 real_ delta2 = _->delta2;
 real_ psi1 = _->psi1;
 real_ psi2 = _->psi2;
 real_ phi1 = _->phi1;
 real_ phi2 = _->phi2;

 real_ amp1 = _->amp1;
 real_ amp2 = _->amp2;
 real_ amp3 = _->amp3;
 real_ amp4 = _->amp4;
 real_ amp5 = _->amp5;
 real_ amp6 = _->amp6;
 real_ deltaamp1 = _->deltaamp1;
 real_ deltaamp2 = _->deltaamp2;
 real_ psiamp1 = _->psiamp1;
 real_ psiamp2 = _->psiamp2;
 real_ phiamp1 = _->phiamp1;
 real_ phiamp2 = _->phiamp2;

 real_ speed1 = _->speed1;
 real_ speed2 = _->speed2;
 real_ speed3 = _->speed3;
 real_ speed4 = _->speed4;
 real_ speed5 = _->speed5;
 real_ speed6 = _->speed6;
 real_ deltaspeed1 = _->deltaspeed1;
 real_ deltaspeed2 = _->deltaspeed2;
 real_ psispeed1 = _->psispeed1;
 real_ psispeed2 = _->psispeed2;
 real_ phispeed1 = _->phispeed1;
 real_ phispeed2 = _->phispeed2;

 coordin angles = {amp1*sin(speed1*_t),amp2*sin(speed2*_t),amp3*sin(speed3*_t)};
 coordin shift = {amp4*sin(speed4*_t),amp5*sin(speed5*_t),amp6*sin(speed6*_t)};

 delta1 = deltaamp1*sin(deltaspeed1*_t);
 delta2 = deltaamp2*sin(deltaspeed2*_t);
 psi1 = psiamp1*sin(psispeed1*_t);
 psi2 = psiamp2*sin(psispeed2*_t);
 phi1 = phiamp1*sin(phispeed1*_t);
 phi2 = phiamp2*sin(phispeed2*_t);

 real_ eta1 = _->eta1;
 real_ eta2 = _->eta2;
 real_ eta3 = _->eta3;
 real_ eta4 = _->eta4;
 real_ eta5 = _->eta5;
 real_ eta6 = _->eta6;

 coordin leg1 = {0.4,0.2,-0.3};
 coordin leg2 = {0.4,-0.2,-0.3};
 coordin leg3 = {0.4,0,-0.25};
 coordin leg4 = {0.4,-0,-0.25};
 coordin leg5 = {0.4,0,-0.25};
 coordin leg6 = {0.4,-0,-0.25};

/*
 Step1(1,a,b,_t,W,leg1);
 leg1[1] = -leg1[1];

 Step1(2,a,b,_t,W,leg2);

 Step1(2,a,b,_t,W,leg3);
 leg3[1] = -leg3[1];

 Step1(1,a,b,_t,W,leg4);

 Step1(1,a,b,_t,W,leg5);
 leg5[1] = -leg5[1];

 Step1(2,a,b,_t,W,leg6);
*/



//вычисление сдвигов корпуса, изломов корпуса и опорных точек ног
real_ local_time = _t-(int)(_t/(2*W))*(2*W);
int number = (int)(_t/(2*W));

if(number==0)
{
if(local_time<=W)
{
shift[1] = 0.5/W*local_time;
}
else
{
delta1 = -1.5707963267948966192313216916398/W*(local_time-W);
shift[1] = 0.5;
}
}

if(number==1)
{
if(local_time<=W)
{
delta1 = -1.5707963267948966192313216916398;
shift[1] = 0.5+0.5/W*local_time;
leg3[1] = 0.5/W*local_time;
leg4[1] = -0.5/W*local_time;
leg5[1] = 0.5/W*local_time;
leg6[1] = -0.5/W*local_time;
}
else
{
delta1 = -1.5707963267948966192313216916398;
shift[1] = 0.5+0.5;
leg3[1] = 0.5;
leg4[1] = -0.5;
leg5[1] = 0.5;
leg6[1] = -0.5;
}
}

if(number==2)
{
if(local_time<=W)
{
delta1 = -1.5707963267948966192313216916398;
shift[1] = 0.5+0.5;
leg3[1] = 0.5+0.3/W*local_time;
leg4[1] = -0.5-0.3/W*local_time;
leg3[2] = -0.25+0.1/W*local_time;
leg4[2] = -0.25+0.1/W*local_time;
leg5[1] = 0.5;
leg6[1] = -0.5;
}
else
{
delta1 = -1.5707963267948966192313216916398;
shift[1] = 0.5+0.5-0.1/W*(local_time-W);
leg3[1] = 0.5+0.3;
leg4[1] = -0.5-0.3;
leg3[2] = -0.25+0.1-0.14/W*(local_time-W);
leg4[2] = -0.25+0.1-0.14/W*(local_time-W);
leg5[1] = 0.5;
leg6[1] = -0.5;
}
}

if(number==3)
{
if(local_time<=W)
{
delta1 = -1.5707963267948966192313216916398;
shift[1] = 0.5+0.5-0.1;
leg3[1] = 0.5+0.3-0.1/W*local_time;
leg4[1] = -0.5-0.3+0.1/W*local_time;
leg3[2] = -0.25+0.1-0.14;
leg4[2] = -0.25+0.1-0.14;
leg5[1] = 0.5;
leg6[1] = -0.5;
}
else
{
delta1 = -1.5707963267948966192313216916398;
shift[1] = 0.5+0.5-0.1;
leg3[1] = 0.5+0.3-0.1;
leg4[1] = -0.5-0.3+0.1;
leg3[2] = -0.25+0.1-0.14;
leg4[2] = -0.25+0.1-0.14;
leg5[1] = 0.5;
leg6[1] = -0.5;
}
}

if(number==4)
{
if(local_time<=W)
{
delta1 = -1.5707963267948966192313216916398;
shift[1] = 0.5+0.5-0.1-0.3/W*local_time;

leg1[1] = 0.2+0.1/W*local_time;
leg2[1] = -0.2-0.1/W*local_time;

leg3[1] = 0.5+0.3-0.1;
leg4[1] = -0.5-0.3+0.1;
leg3[2] = -0.25+0.1-0.14;
leg4[2] = -0.25+0.1-0.14;
leg5[1] = 0.5;
leg6[1] = -0.5;
}
else
{
delta1 = -1.5707963267948966192313216916398;
shift[1] = 0.5+0.5-0.1-0.3;

leg1[1] = 0.2+0.1;
leg2[1] = -0.2-0.1;

leg3[1] = 0.5+0.3-0.1;
leg4[1] = -0.5-0.3+0.1;
leg3[2] = -0.25+0.1-0.14;
leg4[2] = -0.25+0.1-0.14;
leg5[1] = 0.5;
leg6[1] = -0.5;
}
}


//переставляем передние ноги
if(number==5)
{
if(local_time<=W)
{
delta1 = -1.5707963267948966192313216916398;
shift[1] = 0.5+0.5-0.1-0.3;

leg1[0] = 0.4-0.15/W*local_time;
leg2[0] = 0.4-0.15/W*local_time;

leg1[1] = 0.2+0.1;
leg2[1] = -0.2-0.1;

leg1[2] = -0.3-0.6/W*local_time;
leg2[2] = -0.3-0.6/W*local_time;

leg3[1] = 0.5+0.3-0.1;
leg4[1] = -0.5-0.3+0.1;
leg3[2] = -0.25+0.1-0.14;
leg4[2] = -0.25+0.1-0.14;
leg5[1] = 0.5;
leg6[1] = -0.5;
}
else
{
delta1 = -1.5707963267948966192313216916398;
shift[1] = 0.5+0.5-0.1-0.3;

leg1[0] = 0.4-0.15;
leg2[0] = 0.4-0.15;

leg1[1] = 0.2+0.1-0.1/W*(local_time-W);
leg2[1] = -0.2-0.1+0.1/W*(local_time-W);

leg1[2] = -0.3-0.6;
leg2[2] = -0.3-0.6;

leg3[1] = 0.5+0.3-0.1;
leg4[1] = -0.5-0.3+0.1;
leg3[2] = -0.25+0.1-0.14;
leg4[2] = -0.25+0.1-0.14;
leg5[1] = 0.5;
leg6[1] = -0.5;
}
}

if(number==6)
{
if(local_time<=W)
{
delta1 = -1.5707963267948966192313216916398+1.5707963267948966192313216916398/W*local_time;
delta2 = -1.5707963267948966192313216916398/W*local_time;
angles[1]=-1.5707963267948966192313216916398/W*local_time;
shift[1] = 0.5+0.5-0.1-0.3+0.2/W*local_time;
shift[2] = -0.3/(2*W)*local_time;

leg1[0] = 0.4-0.15;
leg2[0] = 0.4-0.15;

leg1[1] = 0.2+0.1-0.1;
leg2[1] = -0.2-0.1+0.1;

leg1[2] = -0.3-0.6;
leg2[2] = -0.3-0.6;

leg3[1] = 0.5+0.3-0.1;
leg4[1] = -0.5-0.3+0.1;
leg3[2] = -0.25+0.1-0.14;
leg4[2] = -0.25+0.1-0.14;
leg5[1] = 0.5;
leg6[1] = -0.5;
}
else
{
delta1 = -1.5707963267948966192313216916398+1.5707963267948966192313216916398;
delta2 = -1.5707963267948966192313216916398;
angles[1]=-1.5707963267948966192313216916398;
shift[1] = 0.5+0.5-0.1-0.3+0.2;
shift[2] = -0.3/(2*W)*(local_time);

leg1[0] = 0.4-0.15;
leg2[0] = 0.4-0.15;

leg1[1] = 0.2+0.1-0.1;
leg2[1] = -0.2-0.1+0.1;

leg1[2] = -0.3-0.6;
leg2[2] = -0.3-0.6;

leg3[1] = 0.5+0.3-0.1;
leg4[1] = -0.5-0.3+0.1;
leg3[2] = -0.25+0.1-0.14;
leg4[2] = -0.25+0.1-0.14;
leg5[1] = 0.5;
leg6[1] = -0.5;
}
}


if(number==7)
{
if(local_time<=W)
{
delta1 = -1.5707963267948966192313216916398+1.5707963267948966192313216916398;
delta2 = -1.5707963267948966192313216916398;
angles[1]=-1.5707963267948966192313216916398;
shift[1] = 0.5+0.5-0.1-0.3+0.2;
shift[2] = -0.3;

leg1[0] = 0.4-0.15;
leg2[0] = 0.4-0.15;

leg1[1] = 0.2+0.1-0.1;
leg2[1] = -0.2-0.1+0.1;

leg1[2] = -0.3-0.6;
leg2[2] = -0.3-0.6;

leg3[1] = 0.5+0.3-0.1;
leg4[1] = -0.5-0.3+0.1;
leg3[2] = -0.25+0.1-0.14;
leg4[2] = -0.25+0.1-0.14;
leg5[1] = 0.5+0.75/W*local_time;
leg6[1] = -0.5-0.75/W*local_time;
}
else
{
delta1 = -1.5707963267948966192313216916398+1.5707963267948966192313216916398;
delta2 = -1.5707963267948966192313216916398;
angles[1]=-1.5707963267948966192313216916398;
shift[1] = 0.5+0.5-0.1-0.3+0.2;
shift[2] = -0.3;

leg1[0] = 0.4-0.15;
leg2[0] = 0.4-0.15;

leg1[1] = 0.2+0.1-0.1;
leg2[1] = -0.2-0.1+0.1;

leg1[2] = -0.3-0.6;
leg2[2] = -0.3-0.6;

leg3[1] = 0.5+0.3-0.1;
leg4[1] = -0.5-0.3+0.1;

leg3[2] = -0.25+0.1-0.14;
leg4[2] = -0.25+0.1-0.14;

leg5[1] = 0.5+0.75;
leg6[1] = -0.5-0.75;

leg5[2] = -0.25-0.04/W*(local_time-W);
leg6[2] = -0.25-0.04/W*(local_time-W);
}
}


if(number==8)
{
if(local_time<=W)
{
delta1 = -1.5707963267948966192313216916398+1.5707963267948966192313216916398;
delta2 = -1.5707963267948966192313216916398;
angles[1]=-1.5707963267948966192313216916398;
shift[1] = 0.5+0.5-0.1-0.3+0.2;
shift[2] = -0.3;

leg1[0] = 0.4-0.15;
leg2[0] = 0.4-0.15;

leg1[1] = 0.2+0.1-0.1;
leg2[1] = -0.2-0.1+0.1;

leg1[2] = -0.3-0.6;
leg2[2] = -0.3-0.6;

leg3[1] = 0.5+0.3-0.1;
leg4[1] = -0.5-0.3+0.1;

leg3[2] = -0.25+0.1-0.14;
leg4[2] = -0.25+0.1-0.14;

leg5[1] = 0.5+0.75-0.05/W*local_time;
leg6[1] = -0.5-0.75+0.05/W*local_time;

leg5[2] = -0.25-0.04;
leg6[2] = -0.25-0.04;
}
else
{
delta1 = -1.5707963267948966192313216916398+1.5707963267948966192313216916398;
delta2 = -1.5707963267948966192313216916398;
angles[1]=-1.5707963267948966192313216916398;
shift[1] = 0.5+0.5-0.1-0.3+0.2;
shift[2] = -0.3;

leg1[0] = 0.4-0.15;
leg2[0] = 0.4-0.15;

leg1[1] = 0.2+0.1-0.1;
leg2[1] = -0.2-0.1+0.1;

leg1[2] = -0.3-0.6;
leg2[2] = -0.3-0.6;

leg3[1] = 0.5+0.3-0.1;
leg4[1] = -0.5-0.3+0.1;

leg3[2] = -0.25+0.1-0.14;
leg4[2] = -0.25+0.1-0.14;

leg5[1] = 0.5+0.75-0.05;
leg6[1] = -0.5-0.75+0.05;

leg5[2] = -0.25-0.04;
leg6[2] = -0.25-0.04;
}
}


//переставляем средние ноги
if(number==9)
{
if(local_time<=W)
{
delta1 = -1.5707963267948966192313216916398+1.5707963267948966192313216916398;
delta2 = -1.5707963267948966192313216916398;
angles[1]=-1.5707963267948966192313216916398;
shift[1] = 0.5+0.5-0.1-0.3+0.2;
shift[2] = -0.3;

leg1[0] = 0.4-0.15;
leg2[0] = 0.4-0.15;

leg1[1] = 0.2+0.1-0.1;
leg2[1] = -0.2-0.1+0.1;

leg1[2] = -0.3-0.6;
leg2[2] = -0.3-0.6;

leg3[1] = 0.5+0.3-0.1+0.1/W*local_time;
leg4[1] = -0.5-0.3+0.1-0.1/W*local_time;

leg3[2] = -0.25+0.1-0.14-0.4/W*local_time;
leg4[2] = -0.25+0.1-0.14-0.4/W*local_time;

leg5[1] = 0.5+0.75-0.05;
leg6[1] = -0.5-0.75+0.05;

leg5[2] = -0.25-0.04;
leg6[2] = -0.25-0.04;
}
else
{
delta1 = -1.5707963267948966192313216916398+1.5707963267948966192313216916398;
delta2 = -1.5707963267948966192313216916398;
angles[1]=-1.5707963267948966192313216916398;
shift[1] = 0.5+0.5-0.1-0.3+0.2;
shift[2] = -0.3;

leg1[0] = 0.4-0.15;
leg2[0] = 0.4-0.15;

leg1[1] = 0.2+0.1-0.1;
leg2[1] = -0.2-0.1+0.1;

leg1[2] = -0.3-0.6;
leg2[2] = -0.3-0.6;

leg3[1] = 0.5+0.3-0.1+0.1-0.1/W*(local_time-W);
leg4[1] = -0.5-0.3+0.1-0.1+0.1/W*(local_time-W);

leg3[2] = -0.25+0.1-0.14-0.4;
leg4[2] = -0.25+0.1-0.14-0.4;

leg5[1] = 0.5+0.75-0.05;
leg6[1] = -0.5-0.75+0.05;

leg5[2] = -0.25-0.04;
leg6[2] = -0.25-0.04;
}
}

//переставляем передние ноги
if(number==10)
{
if(local_time<=W)
{
delta1 = -1.5707963267948966192313216916398+1.5707963267948966192313216916398;
delta2 = -1.5707963267948966192313216916398;
angles[1]=-1.5707963267948966192313216916398;
shift[1] = 0.5+0.5-0.1-0.3+0.2;
shift[2] = -0.3;

leg1[0] = 0.4-0.15+0.15/W*local_time;
leg2[0] = 0.4-0.15+0.15/W*local_time;

leg1[1] = 0.2+0.1-0.1+0.05/W*local_time;
leg2[1] = -0.2-0.1+0.1-0.05/W*local_time;

leg1[2] = -0.3-0.6-0.2/W*local_time;
leg2[2] = -0.3-0.6-0.2/W*local_time;

leg3[1] = 0.5+0.3-0.1+0.1-0.1;
leg4[1] = -0.5-0.3+0.1-0.1+0.1;

leg3[2] = -0.25+0.1-0.14-0.4;
leg4[2] = -0.25+0.1-0.14-0.4;

leg5[1] = 0.5+0.75-0.05;
leg6[1] = -0.5-0.75+0.05;

leg5[2] = -0.25-0.04;
leg6[2] = -0.25-0.04;
}
else
{
delta1 = -1.5707963267948966192313216916398+1.5707963267948966192313216916398;
delta2 = -1.5707963267948966192313216916398;
angles[1]=-1.5707963267948966192313216916398;
shift[1] = 0.5+0.5-0.1-0.3+0.2;
shift[2] = -0.3;

leg1[0] = 0.4-0.15+0.15;
leg2[0] = 0.4-0.15+0.15;

leg1[1] = 0.2+0.1-0.1+0.05-0.05/W*(local_time-W);
leg2[1] = -0.2-0.1+0.1-0.05+0.05/W*(local_time-W);

leg1[2] = -0.3-0.6-0.2;
leg2[2] = -0.3-0.6-0.2;

leg3[1] = 0.5+0.3-0.1+0.1-0.1;
leg4[1] = -0.5-0.3+0.1-0.1+0.1;

leg3[2] = -0.25+0.1-0.14-0.4;
leg4[2] = -0.25+0.1-0.14-0.4;

leg5[1] = 0.5+0.75-0.05;
leg6[1] = -0.5-0.75+0.05;

leg5[2] = -0.25-0.04;
leg6[2] = -0.25-0.04;
}
}

if(number==11)
{
if(local_time<=W)
{
delta1 = -1.5707963267948966192313216916398+1.5707963267948966192313216916398;
delta2 = -1.5707963267948966192313216916398+1.5707963267948966192313216916398/W*local_time;
angles[1]=-1.5707963267948966192313216916398;
shift[1] = 0.5+0.5-0.1-0.3+0.2+0.2/W*local_time;
shift[2] = -0.3-0.5/W*local_time;

leg1[0] = 0.4-0.15+0.15;
leg2[0] = 0.4-0.15+0.15;

leg1[1] = 0.2+0.1-0.1+0.05-0.05;
leg2[1] = -0.2-0.1+0.1-0.05+0.05;

leg1[2] = -0.3-0.6-0.2;
leg2[2] = -0.3-0.6-0.2;

leg3[1] = 0.5+0.3-0.1+0.1-0.1;
leg4[1] = -0.5-0.3+0.1-0.1+0.1;

leg3[2] = -0.25+0.1-0.14-0.4;
leg4[2] = -0.25+0.1-0.14-0.4;

leg5[1] = 0.5+0.75-0.05;
leg6[1] = -0.5-0.75+0.05;

leg5[2] = -0.25-0.04;
leg6[2] = -0.25-0.04;
}
else
{
delta1 = -1.5707963267948966192313216916398+1.5707963267948966192313216916398;
delta2 = -1.5707963267948966192313216916398+1.5707963267948966192313216916398;
angles[1]=-1.5707963267948966192313216916398;
shift[1] = 0.5+0.5-0.1-0.3+0.2+0.2;
shift[2] = -0.3-0.5;

leg1[0] = 0.4-0.15+0.15;
leg2[0] = 0.4-0.15+0.15;

leg1[1] = 0.2+0.1-0.1+0.05-0.05;
leg2[1] = -0.2-0.1+0.1-0.05+0.05;

leg1[2] = -0.3-0.6-0.2;
leg2[2] = -0.3-0.6-0.2;

leg3[1] = 0.5+0.3-0.1+0.1-0.1;
leg4[1] = -0.5-0.3+0.1-0.1+0.1;

leg3[2] = -0.25+0.1-0.14-0.4;
leg4[2] = -0.25+0.1-0.14-0.4;

leg5[1] = 0.5+0.75-0.05;
leg6[1] = -0.5-0.75+0.05;

leg5[2] = -0.25-0.04;
leg6[2] = -0.25-0.04;

}
}

if(number>11)
{
delta1 = -1.5707963267948966192313216916398+1.5707963267948966192313216916398;
delta2 = -1.5707963267948966192313216916398+1.5707963267948966192313216916398;
angles[1]=-1.5707963267948966192313216916398;
shift[1] = 0.5+0.5-0.1-0.3+0.2+0.2;
shift[2] = -0.3-0.5;

leg1[0] = 0.4-0.15+0.15;
leg2[0] = 0.4-0.15+0.15;

leg1[1] = 0.2+0.1-0.1+0.05-0.05;
leg2[1] = -0.2-0.1+0.1-0.05+0.05;

leg1[2] = -0.3-0.6-0.2;
leg2[2] = -0.3-0.6-0.2;

leg3[1] = 0.5+0.3-0.1+0.1-0.1;
leg4[1] = -0.5-0.3+0.1-0.1+0.1;

leg3[2] = -0.25+0.1-0.14-0.4;
leg4[2] = -0.25+0.1-0.14-0.4;

leg5[1] = 0.5+0.75-0.05;
leg6[1] = -0.5-0.75+0.05;

leg5[2] = -0.25-0.04;
leg6[2] = -0.25-0.04;

}

 coordin r1 = {w_body/2,0,0};
 coordin r11 = {0,l_body/2,0};
 coordin r12 = {0,l_body/2*cos(delta1),l_body/2*sin(delta1)};
 coordin r2 = {-w_body/2,0,0};

 coordin r3 = {w_body/2,0,0};
 coordin r4 = {-w_body/2,0,0};

 coordin r5 = {w_body/2,0,0};
 coordin r51 = {0,-l_body/2,0};
 coordin r52 = {0,-l_body/2*cos(delta2),l_body/2*sin(delta2)};
 coordin r6 = {-w_body/2,0,0};

 //пробная часть
 /*
 coordin angle_test = {0,delta1,0};
 coordin leg1_test = {w_body/2,l_body/2,0};
 coordin shift1_test = {0,0,0};

 StepTrans(0,leg1_test,leg1,shift1_test,angle_test);
 */
 coordin angles_leg1 = {delta1,psi1,eta1};
 StepTrans2(angles_leg1,r1,leg1,shift,angles,r11,r12);
 coordin angles1;
 RevKinematics(leg1,p,angles1);
 SetIdentifierValue(9,1,angles1[0]);
 SetIdentifierValue(10,1,angles1[1]);
 SetIdentifierValue(11,1,angles1[2]);

 coordin angles_leg2 = {delta1,psi1,eta2};
 StepTrans2(angles_leg2,r2,leg2,shift,angles,r11,r12);
 coordin angles2;
 RevKinematics(leg2,p,angles2);
 SetIdentifierValue(14,1,angles2[0]);
 SetIdentifierValue(15,1,angles2[1]);
 SetIdentifierValue(16,1,angles2[2]);

 StepTrans(eta3,r3,leg3,shift,angles);
 coordin angles3;
 RevKinematics(leg3,p,angles3);
 SetIdentifierValue(17,1,angles3[0]);
 SetIdentifierValue(18,1,angles3[1]);
 SetIdentifierValue(19,1,angles3[2]);

 StepTrans(eta4,r4,leg4,shift,angles);
 coordin angles4;
 RevKinematics(leg4,p,angles4);
 SetIdentifierValue(20,1,angles4[0]);
 SetIdentifierValue(21,1,angles4[1]);
 SetIdentifierValue(22,1,angles4[2]);

 coordin angles_leg5 = {-delta2,0,eta5};
 StepTrans2(angles_leg5,r5,leg5,shift,angles,r51,r52);
 coordin angles5;
 RevKinematics(leg5,p,angles5);
 SetIdentifierValue(23,1,angles5[0]);
 SetIdentifierValue(24,1,angles5[1]);
 SetIdentifierValue(25,1,angles5[2]);

 coordin angles_leg6 = {-delta2,0,eta6};
 StepTrans2(angles_leg6,r6,leg6,shift,angles,r51,r52);
 coordin angles6;
 RevKinematics(leg6,p,angles6);
 SetIdentifierValue(26,1,angles6[0]);
 SetIdentifierValue(27,1,angles6[1]);
 SetIdentifierValue(28,1,angles6[2]);

 SetIdentifierValue(12,1,delta1);
 SetIdentifierValue(13,1,delta2);

 SetIdentifierValue(62,1,psi1);
 SetIdentifierValue(63,1,psi2);

 SetIdentifierValue(64,1,phi1);
 SetIdentifierValue(65,1,phi2);

 SetIdentifierValue(66,1,number);
 SetIdentifierValue(67,1,local_time);

 }

void ForceFuncCalc( real_ _t, VectRPtr _x, VectRPtr _v, integer _isubs )
 {
  _hex_articulated_climb2VarPtr _;
  _ = _PzAll[SubIndx[_isubs-1]-1];
 }

void Get1stOrderODE( real_ _t, VectRPtr _x, VectRPtr _f, integer _isubs )
 {
  _hex_articulated_climb2VarPtr _;
  _ = _PzAll[SubIndx[_isubs-1]-1];
 }

void Get2ndOrderODE( real_ _t, VectRPtr _x, VectRPtr _v, VectRPtr _f, integer _isubs )
 {
  _hex_articulated_climb2VarPtr _;
  _ = _PzAll[SubIndx[_isubs-1]-1];
 }

void UserConCalc( VectRPtr _x, VectRPtr _v, MatrRPtr _Jacobi, Vec3RPtr _Error, integer _isubs, integer _ic, boolean _predict, integer _nright )
 {
  _hex_articulated_climb2VarPtr _;
  _ = _PzAll[SubIndx[_isubs-1]-1];
 }

void UserCalc( VectRPtr _x, VectRPtr _v, VectRPtr _a, integer _isubs, integer _UMMessage, integer &WhatDo )
 {
  integer Key;
  Key = WhatDo;
  WhatDo = NOTHING;
  switch( _UMMessage )
   {
    case FORCESCALC_MESSAGE :
     {
      try
       {
        ForceFuncCalc( t, _x, _v, _isubs );
       }
      catch(...)
       {
        WhatDo = -1;
       }
      break;
     }
   }
 }

void ControlPanelMessage( VectRPtr _x, VectRPtr _v, VectRPtr _a, integer _isubs, integer _index, double _Value )
 {
  _hex_articulated_climb2VarPtr _;
  _ = _PzAll[SubIndx[_isubs-1]-1];
 }

/* end of file */
#endif
