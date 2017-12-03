// UMLinearSpringC.cpp : Defines the exported functions for the DLL application.
//


#ifdef _MSC_VER
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "stdafx.h"
#include <math.h>

typedef double*  PDouble;
typedef wchar_t*  WChar;

#include <robot/robot.h>

Robot myRobot;

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

  wcscpy(name, L"Simple hexapod test" );
  status = 0;
}

void _cdecl EXT_GetNumU(int& num, int& status)
{
  // num is the count of input signals
  // status is the exit code 
  //   status==0 means no errors occured

  num    = 0;
  status = 0;
}

void _cdecl EXT_GetUName( int i, WChar name, int& status)
{
  // i is the index of input signals, the first signal has index 0
  // name is the input signal to return
  // status is the exit code 
  //   status==0 means no errors occured

  status = 0;
}

void _cdecl EXT_GetNumY(int &num, int& status)
{
  // num is the count of output signals
  // status is the exit code 
  //   status==0 means no errors occured

  num    = 18;
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
  case 0: wcscpy(name, L"FL_alpha"); break;
  case 1: wcscpy(name, L"FL_beta"); break;
  case 2: wcscpy(name, L"FL_gamma"); break;

  case 3: wcscpy(name, L"ML_alpha"); break;
  case 4: wcscpy(name, L"ML_beta"); break;
  case 5: wcscpy(name, L"ML_gamma"); break;

  case 6: wcscpy(name, L"RL_alpha"); break;
  case 7: wcscpy(name, L"RL_beta"); break;
  case 8: wcscpy(name, L"RL_gamma"); break;

  case  9: wcscpy(name, L"FR_alpha"); break;
  case 10: wcscpy(name, L"FR_beta"); break;
  case 11: wcscpy(name, L"FR_gamma"); break;

  case 12: wcscpy(name, L"MR_alpha"); break;
  case 13: wcscpy(name, L"MR_beta"); break;
  case 14: wcscpy(name, L"MR_gamma"); break;

  case 15: wcscpy(name, L"RR_alpha"); break;
  case 16: wcscpy(name, L"RR_beta"); break;
  case 17: wcscpy(name, L"RR_gamma"); break;
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

  //Eigen::VectorXd result = myRobot.getControls(time);
  //
  for(int i = 0; i < 18; i++)
  {
    Y[i] = 1;
  }

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