/* Header file EqUMObject */
#ifndef __EqUMObject_HPP__
#define __EqUMObject_HPP__

/* Part of equations */
/* interface */

#include"CtvSt.hpp"
#include"CtvDll.hpp"

EXTERN_C void cdecl EqCalc( MatrRPtr &_MassMtrx, VectRPtr _x, VectRPtr _v, VectRPtr &_Frc_Vctr, MatrRPtr _FrcJMtrx, integer _isubs );
/* implementation */

#include"UMObjectC.hpp"
#include"ClUMObject.hpp"
#include"_TUMObject.hpp"

void EqCalc( MatrRPtr &_MassMtrx, VectRPtr _x, VectRPtr _v, VectRPtr &_Frc_Vctr, MatrRPtr _FrcJMtrx, integer _isubs )
 {
  _UMObjectVarPtr _;
  _ = _PzAll[SubIndx[_isubs-1]-1];
  if( CommonData->MassMatrixCalculation == 1 )
   {
   }
  if( CommonData->ForcesCalculation == 1 )
   {
    _Frc_Vctr[0] = _->_[387];
    _Frc_Vctr[1] = _->_[396];
    _Frc_Vctr[2] = _->_[402];
    _Frc_Vctr[3] = _->_[406];
    _Frc_Vctr[4] = _->_[410];
    _Frc_Vctr[5] = _->_[414];
    _Frc_Vctr[6] = _->_[415];
   }
  UserPtr1 = _MassMtrx;
  UserPtr2 = _Frc_Vctr;
  UserCalc( _x, _v, NULL, _isubs, EQN_CALC, UserWhatDo );
 }

/* end of file */
#endif
