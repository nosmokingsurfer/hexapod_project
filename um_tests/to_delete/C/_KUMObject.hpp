/* Header file _KUMObject */
#ifndef ___KUMObject_HPP__
#define ___KUMObject_HPP__

/* Procedures for body, joint etc. variables */
/* interface */

#include"CtvSt.hpp"
#include"CtvDll.hpp"

EXTERN_C void cdecl Ai0Calc( trans_matr &_bodyorientation, VectRPtr _x, VectRPtr _v, integer _index, integer _isubs );
EXTERN_C void cdecl CoorCalc( coordin &_bodyposition, VectRPtr _x, VectRPtr _v, integer _index, integer _isubs );
EXTERN_C void cdecl DBCalc( TBDBodyI &_b, VectRPtr _x, VectRPtr _v, integer _index, integer _isubs );
EXTERN_C void cdecl SMatrCalc( TMatrix6x6_ &_slocal, VectRPtr _x, VectRPtr _v, integer _index, integer _isubs );
EXTERN_C void cdecl VelCalc( coordin &_bodyvelocity, VectRPtr _x, VectRPtr _v, integer _index, integer _isubs );
EXTERN_C void cdecl VelaCalc( coordin &_angularvelocity, VectRPtr _x, VectRPtr _v, integer _index, integer _isubs );
EXTERN_C void cdecl AccCalc( coordin &_bodyacceleration, VectRPtr _x, VectRPtr _v, integer _index, integer _isubs );
EXTERN_C void cdecl AccaCalc( coordin &_angularacceleration, VectRPtr _x, VectRPtr _v, integer _index, integer _isubs );
EXTERN_C void cdecl JointPtCalc( coordin &_jointpoint, VectRPtr _x, VectRPtr _v, integer _index, integer _isubs );
EXTERN_C void cdecl SingleAFrcCalc( coordin &_AForce, coordin &_AMoment, VectRPtr _x, VectRPtr _v, integer _index, integer _isubs );
EXTERN_C void cdecl JForceCalc( coordin &_JForce, coordin &_JMoment, VectRPtr _x, VectRPtr _v, integer _index, integer _isubs );
EXTERN_C void cdecl SingleJointFrc( coordin &_JForce, coordin &_JMoment, VectRPtr _x, VectRPtr _v, integer _index, integer _isubs );
EXTERN_C void cdecl ExtFrcCalc( coordin &_ExtForce, coordin &_ExtMoment, VectRPtr _x, VectRPtr _v, integer _index, integer _isubs );
EXTERN_C real_ cdecl CalcBiFrc( integer _index, integer _isubs, real_ _x, real_ _v, real_ _t );
EXTERN_C void cdecl CalcContFunc( integer _index, integer _isubs, real_ _p1, real_ _p2, real_ &_z, real_ &_dzx, real_ &_dzy );
/* implementation */

#include"UMObjectC.hpp"
#include"ClUMObject.hpp"
#include"_TUMObject.hpp"

void Ai0Calc( trans_matr &_bodyorientation, VectRPtr _x, VectRPtr _v, integer _index, integer _isubs )
 {
  _UMObjectVarPtr _;
  _ = _PzAll[SubIndx[_isubs-1]-1];
  switch( _index )
   {
    case 0 :
     {
      _bodyorientation[0][0] = 1;
      _bodyorientation[0][1] = 0;
      _bodyorientation[0][2] = 0;
      _bodyorientation[1][0] = 0;
      _bodyorientation[1][1] = 1;
      _bodyorientation[1][2] = 0;
      _bodyorientation[2][0] = 0;
      _bodyorientation[2][1] = 0;
      _bodyorientation[2][2] = 1;
      break;
     }
    case 1 :
     {
      _bodyorientation[0][0] = _->_[53];
      _bodyorientation[0][1] = _->_[54];
      _bodyorientation[0][2] = _->_[57];
      _bodyorientation[1][0] = _->_[60];
      _bodyorientation[1][1] = _->_[61];
      _bodyorientation[1][2] = _->_[64];
      _bodyorientation[2][0] = _->_[65];
      _bodyorientation[2][1] = -_->_s2;
      _bodyorientation[2][2] = _->_[66];
      break;
     }
    case 2 :
     {
      _bodyorientation[0][0] = _->_[53];
      _bodyorientation[0][1] = _->_[54];
      _bodyorientation[0][2] = _->_[57];
      _bodyorientation[1][0] = _->_[325];
      _bodyorientation[1][1] = _->_[328];
      _bodyorientation[1][2] = _->_[331];
      _bodyorientation[2][0] = _->_[334];
      _bodyorientation[2][1] = _->_[337];
      _bodyorientation[2][2] = _->_[340];
      break;
     }
   }
 }

void CoorCalc( coordin &_bodyposition, VectRPtr _x, VectRPtr _v, integer _index, integer _isubs )
 {
  _UMObjectVarPtr _;
  _ = _PzAll[SubIndx[_isubs-1]-1];
  switch( _index )
   {
    case 0 :
     {
      _bodyposition[0] = 0;
      _bodyposition[1] = 0;
      _bodyposition[2] = 0;
      break;
     }
    case 1 :
     {
      _bodyposition[0] = _->_[11];
      _bodyposition[1] = _->_[41];
      _bodyposition[2] = _->_[47];
      break;
     }
    case 2 :
     {
      _bodyposition[0] = _->_[11];
      _bodyposition[1] = _->_[41];
      _bodyposition[2] = _->_[47];
      break;
     }
   }
 }

void DBCalc( TBDBodyI &_b, VectRPtr _x, VectRPtr _v, integer _index, integer _isubs )
 {
  _UMObjectVarPtr _;
  _ = _PzAll[SubIndx[_isubs-1]-1];
  switch( _index )
   {
    case 1 :
     {
      _b[0][0] = _->_[47];
      _b[1][0] = 0;
      _b[2][0] = -_->_[11];
      _b[3][0] = _->_[54];
      _b[4][0] = _->_[61];
      _b[5][0] = -_->_s2;
      _b[0][1] = _->_[188];
      _b[1][1] = -_->_[5];
      _b[2][1] = _->_[197];
      _b[3][1] = _->_c3;
      _b[4][1] = -_->_s3;
      _b[5][1] = 0;
      _b[0][2] = _->_[244];
      _b[1][2] = _->_[247];
      _b[2][2] = _->_[258];
      _b[3][2] = 0;
      _b[4][2] = 0;
      _b[5][2] = 1;
      _b[0][3] = _->_[65];
      _b[1][3] = -_->_s2;
      _b[2][3] = _->_[66];
      _b[3][3] = 0;
      _b[4][3] = 0;
      _b[5][3] = 0;
      _b[0][4] = _->_[60];
      _b[1][4] = _->_[61];
      _b[2][4] = _->_[64];
      _b[3][4] = 0;
      _b[4][4] = 0;
      _b[5][4] = 0;
      _b[0][5] = _->_[53];
      _b[1][5] = _->_[54];
      _b[2][5] = _->_[57];
      _b[3][5] = 0;
      _b[4][5] = 0;
      _b[5][5] = 0;
      break;
     }
    case 2 :
     {
      _b[0][0] = _->_[47];
      _b[1][0] = 0;
      _b[2][0] = -_->_[11];
      _b[3][0] = _->_[54];
      _b[4][0] = _->_[328];
      _b[5][0] = _->_[337];
      _b[0][1] = _->_[188];
      _b[1][1] = -_->_[5];
      _b[2][1] = _->_[197];
      _b[3][1] = _->_c3;
      _b[4][1] = -_->_[345];
      _b[5][1] = _->_[346];
      _b[0][2] = _->_[244];
      _b[1][2] = _->_[247];
      _b[2][2] = _->_[258];
      _b[3][2] = 0;
      _b[4][2] = _->_s7;
      _b[5][2] = _->_c7;
      _b[0][3] = _->_[65];
      _b[1][3] = -_->_s2;
      _b[2][3] = _->_[66];
      _b[3][3] = 0;
      _b[4][3] = 0;
      _b[5][3] = 0;
      _b[0][4] = _->_[60];
      _b[1][4] = _->_[61];
      _b[2][4] = _->_[64];
      _b[3][4] = 0;
      _b[4][4] = 0;
      _b[5][4] = 0;
      _b[0][5] = _->_[53];
      _b[1][5] = _->_[54];
      _b[2][5] = _->_[57];
      _b[3][5] = 0;
      _b[4][5] = 0;
      _b[5][5] = 0;
      _b[0][6] = 0;
      _b[1][6] = 0;
      _b[2][6] = 0;
      _b[3][6] = 1;
      _b[4][6] = 0;
      _b[5][6] = 0;
      break;
     }
   }
 }

void SMatrCalc( TMatrix6x6_ &_slocal, VectRPtr _x, VectRPtr _v, integer _index, integer _isubs )
 {
  _UMObjectVarPtr _;
  _ = _PzAll[SubIndx[_isubs-1]-1];
  switch( _index )
   {
    case 0 :
     {
      break;
     }
    case 1 :
     {
      _slocal[0][0] = _->_[177];
      _slocal[0][1] = _->_[184];
      _slocal[0][2] = _->_[187];
      _slocal[0][3] = _->_[181];
      _slocal[0][4] = -_->_[174];
      _slocal[0][5] = _->_[170];
      _slocal[1][0] = _->_[208];
      _slocal[1][1] = _->_[220];
      _slocal[1][2] = _->_[232];
      _slocal[1][3] = _->_[211];
      _slocal[1][4] = _->_[223];
      _slocal[1][5] = _->_[235];
      _slocal[2][0] = _->_[264];
      _slocal[2][1] = _->_[267];
      _slocal[2][2] = _->_[270];
      _slocal[2][3] = _->_[65];
      _slocal[2][4] = -_->_s2;
      _slocal[2][5] = _->_[66];
      _slocal[3][0] = _->_[65];
      _slocal[3][1] = -_->_s2;
      _slocal[3][2] = _->_[66];
      _slocal[3][3] = 0;
      _slocal[3][4] = 0;
      _slocal[3][5] = 0;
      _slocal[4][0] = _->_[60];
      _slocal[4][1] = _->_[61];
      _slocal[4][2] = _->_[64];
      _slocal[4][3] = 0;
      _slocal[4][4] = 0;
      _slocal[4][5] = 0;
      _slocal[5][0] = _->_[53];
      _slocal[5][1] = _->_[54];
      _slocal[5][2] = _->_[57];
      _slocal[5][3] = 0;
      _slocal[5][4] = 0;
      _slocal[5][5] = 0;
      break;
     }
    case 2 :
     {
      _slocal[0][0] = _->_[205];
      _slocal[0][1] = _->_[217];
      _slocal[0][2] = _->_[229];
      _slocal[0][3] = _->_[53];
      _slocal[0][4] = _->_[54];
      _slocal[0][5] = _->_[57];
      break;
     }
   }
 }

void VelCalc( coordin &_bodyvelocity, VectRPtr _x, VectRPtr _v, integer _index, integer _isubs )
 {
  _UMObjectVarPtr _;
  _ = _PzAll[SubIndx[_isubs-1]-1];
  switch( _index )
   {
    case 0 :
     {
      _bodyvelocity[0] = 0;
      _bodyvelocity[1] = 0;
      _bodyvelocity[2] = 0;
      break;
     }
    case 1 :
     {
      _bodyvelocity[0] = _->_[38];
      _bodyvelocity[1] = _->_[44];
      _bodyvelocity[2] = _->_[50];
      break;
     }
    case 2 :
     {
      _bodyvelocity[0] = _->_[38];
      _bodyvelocity[1] = _->_[44];
      _bodyvelocity[2] = _->_[50];
      break;
     }
   }
 }

void VelaCalc( coordin &_angularvelocity, VectRPtr _x, VectRPtr _v, integer _index, integer _isubs )
 {
  _UMObjectVarPtr _;
  _ = _PzAll[SubIndx[_isubs-1]-1];
  switch( _index )
   {
    case 0 :
     {
      _angularvelocity[0] = 0;
      _angularvelocity[1] = 0;
      _angularvelocity[2] = 0;
      break;
     }
    case 1 :
     {
      _angularvelocity[0] = _->_[291];
      _angularvelocity[1] = _->_[294];
      _angularvelocity[2] = -_->_[27];
      break;
     }
    case 2 :
     {
      _angularvelocity[0] = _->_[354];
      _angularvelocity[1] = _->_[357];
      _angularvelocity[2] = _->_[360];
      break;
     }
   }
 }

void AccCalc( coordin &_bodyacceleration, VectRPtr _x, VectRPtr _v, integer _index, integer _isubs )
 {
  _UMObjectVarPtr _;
  _ = _PzAll[SubIndx[_isubs-1]-1];
  switch( _index )
   {
    case 0 :
     {
      _bodyacceleration[0] = 0;
      _bodyacceleration[1] = 0;
      _bodyacceleration[2] = 0;
      break;
     }
    case 1 :
     {
      _bodyacceleration[0] = _->_[284];
      _bodyacceleration[1] = _->_[286];
      _bodyacceleration[2] = _->_[288];
      break;
     }
    case 2 :
     {
      _bodyacceleration[0] = _->_[284];
      _bodyacceleration[1] = _->_[286];
      _bodyacceleration[2] = _->_[288];
      break;
     }
   }
 }

void AccaCalc( coordin &_angularacceleration, VectRPtr _x, VectRPtr _v, integer _index, integer _isubs )
 {
  _UMObjectVarPtr _;
  _ = _PzAll[SubIndx[_isubs-1]-1];
  switch( _index )
   {
    case 0 :
     {
      _angularacceleration[0] = 0;
      _angularacceleration[1] = 0;
      _angularacceleration[2] = 0;
      break;
     }
    case 1 :
     {
      _angularacceleration[0] = _->_[192];
      _angularacceleration[1] = _->_[196];
      _angularacceleration[2] = _->_[261];
      break;
     }
    case 2 :
     {
      _angularacceleration[0] = _->_[347];
      _angularacceleration[1] = _->_[350];
      _angularacceleration[2] = _->_[353];
      break;
     }
   }
 }

void JointPtCalc( coordin &_jointpoint, VectRPtr _x, VectRPtr _v, integer _index, integer _isubs )
 {
  _UMObjectVarPtr _;
  _ = _PzAll[SubIndx[_isubs-1]-1];
  switch( _index )
   {
    case 1 :
     {
      _jointpoint[0] = 0;
      _jointpoint[1] = 0;
      _jointpoint[2] = 0;
      break;
     }
    case 2 :
     {
      _jointpoint[0] = 0;
      _jointpoint[1] = 0;
      _jointpoint[2] = 0;
      break;
     }
   }
 }

void SingleAFrcCalc( coordin &_AForce, coordin &_AMoment, VectRPtr _x, VectRPtr _v, integer _index, integer _isubs )
 {
  _UMObjectVarPtr _;
  _ = _PzAll[SubIndx[_isubs-1]-1];
  switch( _index )
   {
    case 0 :
     {
      break;
     }
   }
 }

void SingleJointFrc( coordin &_JForce, coordin &_JMoment, VectRPtr _x, VectRPtr _v, integer _index, integer _isubs )
 {
  _UMObjectVarPtr _;
  _ = _PzAll[SubIndx[_isubs-1]-1];
  switch( _index )
   {
    case 1 :
     {
      _JForce[0] = 0;
      _JForce[1] = 0;
      _JForce[2] = 0;
      _JMoment[0] = 0;
      _JMoment[1] = 0;
      _JMoment[2] = 0;
      break;
     }
    case 2 :
     {
      _JForce[0] = 0;
      _JForce[1] = 0;
      _JForce[2] = 0;
      _JMoment[0] = -_->_jf1;
      _JMoment[1] = 0;
      _JMoment[2] = 0;
      break;
     }
   }
 }

void JForceCalc( coordin &_JForce, coordin &_JMoment, VectRPtr _x, VectRPtr _v, integer _index, integer _isubs )
 {
  _UMObjectVarPtr _;
  _ = _PzAll[SubIndx[_isubs-1]-1];
  switch( _index )
   {
    case 1 :
     {
      _JForce[0] = _bForce[_isubs-1][0][0];
      _JForce[1] = _bForce[_isubs-1][0][1];
      _JForce[2] = _bForce[_isubs-1][0][2];
      _JMoment[0] = _->_[375];
      _JMoment[1] = _bMoment[_isubs-1][0][1];
      _JMoment[2] = _bMoment[_isubs-1][0][2];
      break;
     }
    case 2 :
     {
      _JForce[0] = _bForce[_isubs-1][1][0];
      _JForce[1] = _bForce[_isubs-1][1][1];
      _JForce[2] = _bForce[_isubs-1][1][2];
      _JMoment[0] = _->_[376];
      _JMoment[1] = _bMoment[_isubs-1][1][1];
      _JMoment[2] = _bMoment[_isubs-1][1][2];
      break;
     }
   }
 }

/* Output: force and moment reduced to the center of mass of the active body */
/* force is resolved in CS0, moment - in the body-fixed SC */
void ExtFrcCalc( coordin &_ExtForce, coordin &_ExtMoment, VectRPtr _x, VectRPtr _v, integer _index, integer _isubs )
 {
  _UMObjectVarPtr _;
  _ = _PzAll[SubIndx[_isubs-1]-1];
  switch( _index )
   {
    case 0 :
     {
      break;
     }
   }
 }

real_ CalcBiFrc( integer _index, integer _isubs, real_ _x, real_ _v, real_ _t )
 {
  _UMObjectVarPtr _;
  _ = _PzAll[SubIndx[_isubs-1]-1];
  switch( _index )
   {
    case 0 :
     {
      return 0;
      break;
     }
    default :
     {
      return 0;
      break;
     }
   }
 }

void CalcContFunc( integer _index, integer _isubs, real_ _p1, real_ _p2, real_ &_z, real_ &_dzx, real_ &_dzy )
 {
  _UMObjectVarPtr _;
  _ = _PzAll[SubIndx[_isubs-1]-1];
  switch( _index )
   {
    case 0 :
     {
      break;
     }
   }
 }

/* end of file */
#endif
