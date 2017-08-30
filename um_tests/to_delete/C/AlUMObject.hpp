/* Header file AlUMObject */
#ifndef __AlUMObject_HPP__
#define __AlUMObject_HPP__

/* interface */

#include"CtvSt.hpp"
#include"CtvDll.hpp"

EXTERN_C void cdecl AllCalc( VectRPtr _x, VectRPtr _v, integer &_isubs, real_ _alpha, real_ _alpha2 );
/* implementation */

#include"ClUMObject.hpp"
#include"_TUMObject.hpp"
#include"UMObjectC.hpp"
#include"UMObjectE.hpp"
#include"DGetVars.hpp"

void AllCalc( VectRPtr _x, VectRPtr _v, integer &_isubs, real_ _alpha, real_ _alpha2 )
 {
  _UMObjectVarPtr _;
  _ = _PzAll[SubIndx[_isubs-1]-1];
  _GAlpha = _alpha;
  _GAlpha2 = _alpha2;
  if( CommonData->KinemCalculation == 1 )
   {
    _->_ap = CommonData->APredVector;
    try
     {
      TimeFuncCalc( t, _x, _v, _isubs );
     }
    catch(...)
     {
      _isubs = -2;
      exit(1);
     }
    _->_s1 = sin( _x[0] );
    _->_c1 = cos( _x[0] );
    _->_s2 = sin( _x[1] );
    _->_c2 = cos( _x[1] );
    _->_s3 = sin( _x[2] );
    _->_c3 = cos( _x[2] );
    _->_s7 = sin( _x[6] );
    _->_c7 = cos( _x[6] );
    DoElement( _x, _v, 1, _isubs );
   }
  if( CommonData->ForcesCalculation == 1 )
   {
    _->_ap = CommonData->APredVector;
    PredictorClc = true;
    InitJFrc( _isubs, 1, _->_jf1 );
    PredictorClc = false;
    DoElement( _x, _v, 2, _isubs );
   }
  if( CommonData->MassMatrixCalculation == 1 )
   {
    _lMassMatrix = CommonData->MMatrixPtr;
    DoElement( _x, _v, 3, _isubs );
    DoElement( _x, _v, 4, _isubs );
    UserPtr1 = _lMassMatrix;
    UserCalc( _x, _v, NULL, _isubs, CALCMASSMATRIX_MESSAGE, UserWhatDo );
   }
 }

/* end of file */
#endif
