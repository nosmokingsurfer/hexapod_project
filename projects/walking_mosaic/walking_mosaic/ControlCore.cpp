// UMLinearSpringC.cpp : Defines the exported functions for the DLL application.
//


#ifdef _MSC_VER
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "stdafx.h"
#include <math.h>
#include <leg.h>

typedef double*  PDouble;
typedef wchar_t*  WChar;

double PID_coeffs[3]; // model parameters are linear stiffness in each direction, N/m

Leg leg1;


void _cdecl EXT_Initialize( int& status)
{
  // Here we should alloc arrays, create classes and initialize all variables
  // status is the exit code 
  //   status==0 means no errors occured

  // Initial values of parameters
  PID_coeffs[0] = 0;
  PID_coeffs[1] = 0;
  PID_coeffs[2] = 0;
  
  leg1.init(Eigen::Vector3d(1,2,3), Eigen::Vector4d(0,0,0,1), Eigen::Vector3d(0,0,0));


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

  wcscpy(name, L"Leg Test" );
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
  case 0: wcscpy(name, L"alpha"); break;
  case 1: wcscpy(name, L"beta"); break;
  case 2: wcscpy(name, L"gamma"); break;
  case 3: wcscpy(name, L"V_alpha"); break;
  case 4: wcscpy(name, L"V_beta"); break;
  case 5: wcscpy(name, L"V_gamma"); break;
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
  case 0: wcscpy(name, L"torque_alpha"); break;
  case 1: wcscpy(name, L"torque_beta"); break;
  case 2: wcscpy(name, L"torque_gamma"); break;
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

  Eigen::Vector3d traj = leg1.trajectoryGenerator(time);


  Eigen::Vector3d angles;
  leg1.inverseKinematics(traj, angles);
 

  double alpha_goal = angles[0];
  double beta_goal = angles[1];
  double gamma_goal = angles[2];



  Y[0] = PID_coeffs[0]*(alpha_goal - U[0]) + PID_coeffs[2]*(-U[3]);
  Y[1] = PID_coeffs[0]*(beta_goal - U[1]) + PID_coeffs[2]*(-U[4]);
  Y[2] = PID_coeffs[0]*(gamma_goal - U[2]) + PID_coeffs[2]*(-U[5]);


  status = 0;
}

void _cdecl EXT_GetNumParameters(int& num, int& status)
{
  // num - count of model parameters
  // status - exit code (0 - no errors)

  num    = 3;    // Count of model parameters
  status = 0;
}

void _cdecl EXT_GetParameters(PDouble value, int& status)
{
  // value is the pointer to the first element of vector of parameters
  // status is the exit code 
  //   status==0 means no errors occured

  for(int i = 0; i < 3; i++)
    value[i] = PID_coeffs[i];
  
  status = 0;
}

void _cdecl EXT_GetParameterName(int i, WChar name, int& status)
{
  // i is the index of parameter, first parameter has index 0
  // name is the name of parameter
  // status is the exit code 
  //   status==0 means no errors occured

  status = 0;
  switch( i ) 
  {
  case 0: wcscpy(name, L"P"); break;
  case 1: wcscpy(name, L"I"); break;
  case 2: wcscpy(name, L"D"); break;
  default: status = 1;
  };
}

void _cdecl EXT_SetParameters(int numpara, PDouble para, int& status)
{
  // numpara is the length of vector of parameters
  // para is the pointer to the first element of vector of parameter
  // status is the exit code 
  //   status==0 means no errors occured

  for(int i = 0; i < 3; i++)
    PID_coeffs[i] = para[i];
  
  status = 0;
}

void _cdecl EXT_StepConfirmed()
{
  // Universal Mechanism uses numerical integration methods with a variable step
  // size. If this external library includes built-in intergation method it
  // should use this procedure to shift current time.
}