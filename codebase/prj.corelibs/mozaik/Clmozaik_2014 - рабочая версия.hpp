/* Header file Clmozaik_2014 */
#ifndef __Clmozaik_2014_HPP__
#define __Clmozaik_2014_HPP__

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
#include"mozaik_2014C.hpp"
#include"_Tmozaik_2014.hpp"

struct Joint{
int ibody;
int isubs;
coordin connection_point;
coordin orientation angles;
};


int MyVectorIndex1;
int MyVectorIndex2;
int MyVectorIndex3;
int MyVectorIndex4;
int MyVectorIndex5;
int MyVectorIndex6;
int MyVectorIndex7;
int MyVectorIndex8;
int MyVectorIndex9;
int MyVectorIndex10;
int MyVectorIndex11;
int MyVectorIndex12;
int MyVectorIndex13;

int measured = 0;

#include <stdio.h>

FILE* f = fopen("debug.txt","w");

void PrintMatrix(trans_matr A, char* name)
{
       fprintf(f,"Matrix ");
       fprintf(f,name);
       fprintf(f,":\n");
       fprintf(f,"%lf\t%lf\t%lf\n",A[0][0],A[0][1],A[0][2]);
       fprintf(f,"%lf\t%lf\t%lf\n",A[1][0],A[1][1],A[1][2]);
       fprintf(f,"%lf\t%lf\t%lf\n\n\n",A[2][0],A[2][1],A[2][2]);
}

void PrintVector(coordin A, char* name)
{
       fprintf(f,"Vector ");
       fprintf(f,name);
       fprintf(f,": %lf\t%lf\t%lf\n\n",A[0],A[1],A[2]);
}

void PrintScalar(double A, char* name)
{
       fprintf(f,"Parameter ");
       fprintf(f,name);
       fprintf(f,": %lf\n\n",A);
}


void PrintLine(char* line)
{
       fprintf(f,line);
       fprintf(f,"\n");
}

void Angles(trans_matr A, coordin a)
{
    a[0] = atan2(-A[0][1],A[1][1]);
    a[1] = atan2(cos(a[0])*A[2][1],A[1][1]);
    a[2] = atan2(-A[2][0],A[2][2]);
}


/* Функция "TimeFuncCalc" используется исключительно для расчета функций времени.
Не используйте функции типа "GetPoint" внутри этой процедуры.
Недопустим расчет сил в этой процедуре. */
void TimeFuncCalc( real_ _t, VectRPtr _x, VectRPtr _v, integer _isubs )
 {
  _mozaik_2014VarPtr _;
  _ = _PzAll[SubIndx[_isubs-1]-1];
 }

void ForceFuncCalc( real_ _t, VectRPtr _x, VectRPtr _v, integer _isubs )
 {
  _mozaik_2014VarPtr _;
  _ = _PzAll[SubIndx[_isubs-1]-1];
 }

void Get1stOrderODE( real_ _t, VectRPtr _x, VectRPtr _f, integer _isubs )
 {
  _mozaik_2014VarPtr _;
  _ = _PzAll[SubIndx[_isubs-1]-1];
 }

void Get2ndOrderODE( real_ _t, VectRPtr _x, VectRPtr _v, VectRPtr _f, integer _isubs )
 {
  _mozaik_2014VarPtr _;
  _ = _PzAll[SubIndx[_isubs-1]-1];
 }

void UserConCalc( VectRPtr _x, VectRPtr _v, MatrRPtr _Jacobi, Vec3RPtr _Error, integer _isubs, integer _ic, boolean _predict, integer _nright )
 {
  _mozaik_2014VarPtr _;
  _ = _PzAll[SubIndx[_isubs-1]-1];
 }

void UserCalc( VectRPtr _x, VectRPtr _v, VectRPtr _a, integer _isubs, integer _UMMessage, integer &WhatDo )
 {
  integer Key;
  Key = WhatDo;
  WhatDo = NOTHING;

  _mozaik_2014VarPtr _;
  _ = _PzAll[SubIndx[_isubs-1]-1];

  coordin J1;
  J1[0] = 0;
  J1[1] = 1.4142135623730950488016887242097*_->a/2;
  J1[2] = _->c/2;

  coordin J2;
  J2[0] = 1.4142135623730950488016887242097*_->a/2;
  J2[1] = 0;
  J2[2] = _->c/2;

  coordin J3;
  J3[0] = 0;
  J3[1] = -1.4142135623730950488016887242097*_->a/2;
  J3[2] = _->c/2;

  coordin J4;
  J4[0] = -1.4142135623730950488016887242097*_->a/2;
  J4[1] = 0;
  J4[2] = _->c/2;

  coordin J5;
  J5[0] = 0;
  J5[1] = 1.4142135623730950488016887242097*_->a/2;
  J5[2] = _->c/2;

  coordin J6;
  J6[0] = 1.4142135623730950488016887242097*_->a/2;
  J6[1] = 0;
  J6[2] = _->c/2;

  coordin J7;
  J7[0] = 0;
  J7[1] = -1.4142135623730950488016887242097*_->a/2;
  J7[2] = _->c/2;

  coordin zero;
  zero[0] = 0;
  zero[1] = 0;
  zero[2] = 0;

  switch( _UMMessage )
   {
    case STEPEND_MESSAGE :
     {

     }
    case FORCESCALC_MESSAGE :
     {
      try
       {

       coordin start;
       coordin end;

       trans_matr A1;
       trans_matr A2;
       trans_matr A3;
       trans_matr A4;
       trans_matr A5;
       trans_matr A6;

       coordin axis_J1J3;
       coordin axis_J2J4;
       coordin axis_J5J7;

       double J1J3_module;
       double J2J4_module;
       double J5J7_module;

       coordin axis_J1J3_in_1;
       coordin axis_J1J3_in_3;

       coordin axis_J2J4_in_2;

       coordin axis_J5J7_in_5;

       trans_matr A21;
       trans_matr A32;
       trans_matr A43;
       trans_matr A52;

       trans_matr new_A21;
       trans_matr new_A32;
       trans_matr new_A43;
       trans_matr new_A52;

       trans_matr rot;
       trans_matr R;
       coordin eZ;

if((t>2)&&(measured == 1))
{
       //находим радиус-вектор точки J1
       GetPoint(1,1,J1,start);
       start[0] = start[0]*5.0;
       start[1] = start[1]*5.0;
       start[2] = start[2]*5.0;
       SetVectorValue(MyVectorIndex1,start,zero);

       //находим радиус-вектор точки J3
       GetPoint(3,1,J3,end);
       end[0] = end[0]*5;
       end[1] = end[1]*5;
       end[2] = end[2]*5;
       SetVectorValue(MyVectorIndex3,end,zero);

       //Находим значение вектора оси J1J3
       axis_J1J3[0] = end[0]-start[0];
       axis_J1J3[1] = end[1]-start[1];
       axis_J1J3[2] = end[2]-start[2];

       start[0] = start[0]/5.0;
       start[1] = start[1]/5.0;
       start[2] = start[2]/5.0;

       //строим ось J1J3
       SetVectorValue(MyVectorIndex5,axis_J1J3,start);

       //нормировка вектора J1J3

       J1J3_module = sqrt(pow(axis_J1J3[0],2) + pow(axis_J1J3[1],2) + pow(axis_J1J3[2],2));
       for(int i =0; i<3 ;i++)
       {
          axis_J1J3[i]/=J1J3_module;
       }
       //считываем показания один раз
       measured++;

       double alpha = _->J1J3;//угол поворота вокруг оси J1J3
       PrintScalar(alpha,"Alpha");

       PrintLine("Считаем углы для шарнира J1");
//считаем углы для шарнира J1

       GetAi0(1,1,A1);//матрица ориентации первого сегмента в глобальной системе координат
       PrintMatrix(A1,"A1 J1J3");
       Mult_m_v_3x1(NORMAL,A1,axis_J1J3,axis_J1J3_in_1);//ось вращения в СК первого сегмента
       GetAi0(2,1,A2);//матрица ориентации второго сегмента в глобальной системе координат
       PrintMatrix(A2,"A2 J1J3");
       Mult_mT_m(A1,A2,A21);//матрица перехода из СК второго сегмента в СК первого сегмента
       PrintMatrix(A21,"A21 J1J3");

       //выполняем поворот вокруг оси J1J3 на угол alpha


       //проверка оси вращения
       Mult_m_v_3x1(TRANSPON,A1,axis_J1J3_in_1,axis_J1J3);
       PrintVector(axis_J1J3,"Axis J1J3");
       SetVectorValue(MyVectorIndex7,axis_J1J3,start);

       Turning(alpha,axis_J1J3_in_1,rot);//инициация матрици поворота вокруг оси J1J3
       PrintMatrix(rot,"Turning by UM");
       /*
       rot[0][0] = cos(alpha)+(1-cos(alpha))*pow(axis_J1J3_in_1[0],2);
       rot[0][1] = (1-cos(alpha))*axis_J1J3_in_1[0]*axis_J1J3_in_1[1]+sin(alpha)*axis_J1J3_in_1[2];
       rot[0][2] = (1-cos(alpha))*axis_J1J3_in_1[0]*axis_J1J3_in_1[2]-sin(alpha)*axis_J1J3_in_1[1];

       rot[1][0] = (1-cos(alpha))*axis_J1J3_in_1[0]*axis_J1J3_in_1[1]-sin(alpha)*axis_J1J3_in_1[2];
       rot[1][1] = cos(alpha)+(1-cos(alpha))*pow(axis_J1J3_in_1[1],2);
       rot[1][2] = (1-cos(alpha))*axis_J1J3_in_1[1]*axis_J1J3_in_1[2]+sin(alpha)*axis_J1J3_in_1[0];

       rot[2][0] = (1-cos(alpha))*axis_J1J3_in_1[2]*axis_J1J3_in_1[0]+sin(alpha)*axis_J1J3_in_1[1];
       rot[2][1] = (1-cos(alpha))*axis_J1J3_in_1[2]*axis_J1J3_in_1[1]-sin(alpha)*axis_J1J3_in_1[0];
       rot[2][2] = cos(alpha)+(1-cos(alpha))*pow(axis_J1J3_in_1[2],2);

       PrintMatrix(rot,"Turning by me");
       */

       eZ[0] = 0;
       eZ[1] = 0;
       eZ[2] = 1;

       Turning(pi/4,eZ,R);
       PrintMatrix(R,"eZ rotation pi/4 J1");


       Mult_mT_mT(rot,A21,new_A21);//новая матрица перехода из СК второго сегмента в СК первого сегмента
       PrintMatrix(new_A21,"new_A21 before additional rotation");


       //для данного вида шарнира требуется домножить new_A21 на обратные матрицы постоянных
       //поворотов на pi/4 из обобщенного шарнира, чтобы локализовать матрицы поворота углов
       //Крылова для корректного нахождения новых шарнирных углов
       Mult_mT_m(R,new_A21,new_A21);
       Mult_m_m(new_A21,R,new_A21);

       PrintMatrix(new_A21,"New A21");
/*
       real_ psi2,theta2,phi2;
       psi2 = atan2(-new_A21[0][1],new_A21[1][1]);
       theta2 = atan2(cos(psi2)*new_A21[2][1],new_A21[1][1]);
       phi2 = atan2(-new_A21[2][0],new_A21[2][2]);
*/
       coordin angles2;
       Angles(new_A21, angles2);

       SetIdentifierValue(6,1,angles2[0]);
       SetIdentifierValue(7,1,angles2[1]);
       SetIdentifierValue(8,1,angles2[2]);


//выполняем поворот для шарнира J3
       PrintLine("Считаем углы для шарнира J3");

       GetAi0(3,1,A3);//матрица ориентации третьего сегмента в глобальной системе координат
       PrintMatrix(A3,"A3 J1J3");
       Mult_m_v_3x1(NORMAL,A3,axis_J1J3,axis_J1J3_in_3);//ось вращения в СК третьего сегмента
       GetAi0(4,1,A4);//матрица ориентации четвертого сегмента в глобальной системе координат
       PrintMatrix(A4,"A4 J1J3");
       Mult_mT_m(A3,A4,A43);//матрица перехода из СК четвертого сегмента в СК третьего сегмента
       PrintMatrix(A43,"A43");

       //выполняем поворот вокруг оси J1J3 на угол alpha
       //проверка оси вращения
       Mult_m_v_3x1(TRANSPON,A3,axis_J1J3_in_3,axis_J1J3);
       PrintVector(axis_J1J3,"Axis J1J3");
       SetVectorValue(MyVectorIndex8,axis_J1J3,end);

       Turning(alpha,axis_J1J3_in_3,rot);//инициация матрици поворота вокруг оси J1J3
       PrintMatrix(rot,"Turning by UM");

       Mult_m_mT(rot,A43,new_A43);//новая матрица перехода из СК второго сегмента в СК первого сегмента
       PrintMatrix(new_A43,"new_A43 before additional rotation");


       //для данного вида шарнира требуется домножить new_A21 на обратные матрицы постоянных
       //поворотов на pi/4 из обобщенного шарнира, чтобы локализовать матрицы поворота углов
       //Крылова для корректного нахождения новых шарнирных углов

       eZ[0] = 0;
       eZ[1] = 0;
       eZ[2] = 1;

       Turning(-3*pi/4,eZ,R);
       PrintMatrix(R,"eZ rotation pi/4+pi J3");

       Mult_mT_m(R,new_A43,new_A43);
       Mult_m_m(new_A43,R,new_A43);

       PrintMatrix(new_A43,"New A43");
       real_ psi4,theta4,phi4;
       psi4 = atan2(-new_A43[0][1],new_A43[1][1]);
       theta4 = atan2(cos(psi4)*new_A43[2][1],new_A43[1][1]);
       phi4 = atan2(-new_A43[2][0],new_A43[2][2]);

       SetIdentifierValue(12,1,psi4);
       SetIdentifierValue(13,1,theta4);
       SetIdentifierValue(14,1,phi4);

}

//прототип функции нахождения новых шарнирных углов
//Angles(trans_matr Rotation, coordin angles)
//по матрице перехода между СК сегментов находим шарнирные углы



//прототип функции сворачивания корпуса
//Fold(body1 int, body2 int,neighbour1 int, neighbour2 int, joint1 int, joint2 int, double angle, coordin angles1, coordin angles2)
//строим вектор joint1 to joint2 И вокруг этой оси поворачиваем завенья на угол alpha. в переменные angles1 и angles2 записываем новые значения шарнирных углов

if((t>1)&&(measured == 0))
{

       //находим радиус-вектор точки J2
       GetPoint(2,1,J2,start);//координаты шарнира J2 в абсолютной системе координат
       start[0] = start[0]*5.0;
       start[1] = start[1]*5.0;
       start[2] = start[2]*5.0;
       SetVectorValue(MyVectorIndex2,start,zero);//строим радиус вектор к шарниру J2

       //Находим радиус-вектор точки J4
       GetPoint(4,1,J4,end);//положение шарнира J4 в абсолютной системе координат
       end[0] = end[0]*5;
       end[1] = end[1]*5;
       end[2] = end[2]*5;
       SetVectorValue(MyVectorIndex4,end,zero);//строим радиус-вектор к шарниру J4

       //находим значение вектора J2J4
       axis_J2J4[0] = end[0] - start[0];
       axis_J2J4[1] = end[1] - start[1];
       axis_J2J4[2] = end[2] - start[2];

       //масштабируем вектор для корректного построения
       start[0] = start[0]/5.0;
       start[1] = start[1]/5.0;
       start[2] = start[2]/5.0;

       //строим вектор оси J2J4
       SetVectorValue(MyVectorIndex6,axis_J2J4,start);

       J2J4_module = sqrt(pow(axis_J2J4[0],2) + pow(axis_J2J4[1],2) + pow(axis_J2J4[2],2));
       for(int i =0;i<3;i++)
       {
          axis_J2J4[i]/=J2J4_module;
       }


       measured++;
       double beta = _->J2J4;
       PrintScalar(beta,"Angle beta");

//считаем углы для шарнира J2;


       GetAi0(2,1,A2);//матрица ориентации СК второго сегмента
       PrintMatrix(A2,"A2 J2J4");

       Mult_m_v_3x1(NORMAL,A2,axis_J2J4,axis_J2J4_in_2);//ось поворота в СК второго сегмента

       GetAi0(3,1,A3);//матрица ориентации третьего мегмента
       PrintMatrix(A3,"A3 J2J4");

       Mult_mT_m(A2,A3,A32);//матрица перехода от СК третьего сегмента в СК второго сегмента
       PrintMatrix(A32,"A32 J2J4");

       //проверка оси вращения
       Mult_m_v_3x1(TRANSPON,A2,axis_J2J4_in_2,axis_J2J4);
       PrintVector(axis_J2J4,"Axis J2J4");
       SetVectorValue(MyVectorIndex9,axis_J2J4,start);

       Turning(beta,axis_J2J4_in_2,rot);//инициация матрици поворота вокруг оси J2J4
       PrintMatrix(rot,"Turning by UM");

       eZ[0] = 0;
       eZ[1] = 0;
       eZ[2] = 1;
       Turning(pi/4+pi/2,eZ,R);
       PrintMatrix(R,"eZ rotation pi/4+pi/2");


       Mult_mT_mT(rot,A32,new_A32);
       PrintMatrix(new_A32,"new_A32 before additional rotation");

       Mult_mT_m(R,new_A32,new_A32);
       Mult_m_m(new_A32,R,new_A32);

       PrintMatrix(new_A32,"New A32");
       real_ psi3,theta3,phi3;
       psi3 = atan2(-new_A32[0][1],new_A32[1][1]);
       theta3 = atan2(cos(psi3)*new_A32[2][1],new_A32[1][1]);
       phi3 = atan2(-new_A32[2][0],new_A32[2][2]);

       SetIdentifierValue(9,1,psi3);
       SetIdentifierValue(10,1,theta3);
       SetIdentifierValue(11,1,phi3);

       SetIdentifierValue(24,1,psi3);
       SetIdentifierValue(25,1,theta3);
       SetIdentifierValue(26,1,phi3);
}

//строим радиус-векторы шарниров J5 J6 J7
       GetPoint(5,1,J5,start);
       start[0] = start[0]*5.0;
       start[1] = start[1]*5.0;
       start[2] = start[2]*5.0;
       SetVectorValue(MyVectorIndex11,start,zero);


        ForceFuncCalc( t, _x, _v, _isubs );
       }
      catch(...)
       {
        WhatDo = -1;
       }
      break;
     }
    case FIRSTINIT_MESSAGE :
     {
      MyVectorIndex1 = AddUserVector("J1",vtNoType);
      MyVectorIndex2 = AddUserVector("J2",vtNoType);
      MyVectorIndex3 = AddUserVector("J3",vtNoType);
      MyVectorIndex4 = AddUserVector("J4",vtNoType);
      MyVectorIndex5 = AddUserVector("J1J3",vtNoType);
      MyVectorIndex6 = AddUserVector("J2J4",vtNoType);
      MyVectorIndex7 = AddUserVector("RotationAxis J1J3", vtNoType);
      MyVectorIndex8 = AddUserVector("RotationAxis J1J3", vtNoType);
      MyVectorIndex9 = AddUserVector("RotationAxis J2J4", vtNoType);
      MyVectorIndex10 = AddUserVector("RotationAxis J2J4", vtNoType);
      MyVectorIndex11 = AddUserVector("J5",vtNoType);
      MyVectorIndex12 = AddUserVector("J6",vtNoType);
      MyVectorIndex13 = AddUserVector("J7",vtNoType);
     }
    case INTEGRFORM_MESSAGE :
     {
      measured = 0;

       SetIdentifierValue(6,1,0);
       SetIdentifierValue(7,1,0);
       SetIdentifierValue(8,1,0);
       SetIdentifierValue(9,1,0);
       SetIdentifierValue(10,1,0);
       SetIdentifierValue(11,1,0);
       SetIdentifierValue(12,1,0);
       SetIdentifierValue(13,1,0);
       SetIdentifierValue(14,1,0);
       SetIdentifierValue(15,1,0);
       SetIdentifierValue(16,1,0);
       SetIdentifierValue(17,1,0);
       SetIdentifierValue(21,1,0);
       SetIdentifierValue(22,1,0);
       SetIdentifierValue(23,1,0);
       SetIdentifierValue(24,1,0);
       SetIdentifierValue(25,1,0);
       SetIdentifierValue(26,1,0);
       SetIdentifierValue(27,1,0);
       SetIdentifierValue(28,1,0);
       SetIdentifierValue(29,1,0);
     }
   }
 }

void ControlPanelMessage( VectRPtr _x, VectRPtr _v, VectRPtr _a, integer _isubs, integer _index, double _Value )
 {
  _mozaik_2014VarPtr _;
  _ = _PzAll[SubIndx[_isubs-1]-1];
 }

/* end of file */
#endif
