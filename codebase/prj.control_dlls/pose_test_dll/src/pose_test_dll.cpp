// UMLinearSpringC.cpp : Defines the exported functions for the DLL application.
//


#ifdef _MSC_VER
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "stdafx.h"
#include <math.h>

#include <pose/pose.h>

typedef double*  PDouble;
typedef wchar_t*  WChar;

Pose myPose;

Eigen::Vector3d result;

void _cdecl EXT_Initialize( int& status)
{
  // Here we should alloc arrays, create classes and initialize all variables
  // status is the exit code 
  //   status==0 means no errors occured

  // Initial values of parameters
  
  status = 0;
}

void _cdecl EXT_Terminate( int& status )
{
  // Here we should free all arrays and classes and initialize all variables
  // status is the exit code 
  //   status==0 means no errors occured

  status = 0;
}

void _cdecl EXT_GetModelName(WChar name, int& status)
{
  // name is the model name to return
  // status is the exit code 
  //   status==0 means no errors occured

  wcscpy(name, L"Pose angles test" );
  status = 0;
}

void _cdecl EXT_GetNumU(int& num, int& status)
{
  // num is the count of input signals
  // status is the exit code 
  //   status==0 means no errors occured

  num    = 6;
  status = 0;
}

void _cdecl EXT_GetUName( int i, WChar name, int& status)
{
  // i is the index of input signals, the first signal has index 0
  // name is the input signal to return
  // status is the exit code 
  //   status==0 means no errors occured

  status = 0;
  switch( i ) // Names of input variables
  {
  case 0: wcscpy(name, L"yaw"); break;
  case 1: wcscpy(name, L"pitch"); break;
  case 2: wcscpy(name, L"roll"); break;
  case 3: wcscpy(name, L"X"); break;
  case 4: wcscpy(name, L"Y"); break;
  case 5: wcscpy(name, L"Z"); break;
  default: status = 1;
  }
}

void _cdecl EXT_GetNumY(int &num, int& status)
{
  // num is the count of output signals
  // status is the exit code 
  //   status==0 means no errors occured

  num    = 3;
  status = 0;
}

void _cdecl EXT_GetYName(int i, WChar name, int& status)
{
  // i is the index of parameter, the first parameter has index 0
  // name is the output signal name to return
  // status is the exit code 
  //   status==0 means no errors occured

  status = 0;
  switch( i ) // Names of output variables
  {  
  case 0: wcscpy(name, L"X"); break;
  case 1: wcscpy(name, L"Y"); break;
  case 2: wcscpy(name, L"Z"); break;
  default: status = 1;
  }
}

void _cdecl EXT_GetY(double time, PDouble U, PDouble X, PDouble Y, int& status)
{
  // Basic computation procedure that calculates values of output signals based
  // on the current time, input values and values of model parameters
  //
  // Parameters:
  // time is the current model time, sec
  // U is the pointer to the first element of vector of input values
  // X is reserved for future use (state variables)
  // Y is the pointer to the first element of vector of output variables
  // status is the exit code 
  //   status==0 means no errors occured
  double yaw = U[0];
  double pitch = U[1];
  double roll = U[2];

  Eigen::Vector3d goal(0.25, 0.5, 0.2); //положение точки в СК корпуса 

  myPose = Pose(Eigen::Vector3d(yaw, pitch, roll), Eigen::Vector3d(0,0,0)); //ориентация корпуса в абсолютной системе координат

  result = myPose.T.inverse()*goal; // положение точки корпуса в абсолютной системе координат
  
  Y[0] = result[0];
  Y[1] = result[1];
  Y[2] = result[2];

  status = 0;
}

void _cdecl EXT_GetNumParameters(int& num, int& status)
{
  // num - count of model parameters
  // status - exit code (0 - no errors)

  num    = 0;    // Count of model parameters
  status = 0;
}

void _cdecl EXT_GetParameters(PDouble value, int& status)
{
  // value is the pointer to the first element of vector of parameters
  // status is the exit code 
  //   status==0 means no errors occured

  status = 0;
}

void _cdecl EXT_GetParameterName(int i, WChar name, int& status)
{
  // i is the index of parameter, first parameter has index 0
  // name is the name of parameter
  // status is the exit code 
  //   status==0 means no errors occured

  status = 0;
}

void _cdecl EXT_SetParameters(int numpara, PDouble para, int& status)
{
  // numpara is the length of vector of parameters
  // para is the pointer to the first element of vector of parameter
  // status is the exit code 
  //   status==0 means no errors occured


  status = 0;
}

void _cdecl EXT_StepConfirmed()
{
  // Universal Mechanism uses numerical integration methods with a variable step
  // size. If this external library includes built-in intergation method it
  // should use this procedure to shift current time.
}