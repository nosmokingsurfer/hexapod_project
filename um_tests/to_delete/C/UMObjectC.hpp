/* Header file UMObjectC */
#ifndef __UMObjectC_HPP__
#define __UMObjectC_HPP__

/* interface */

#include"CtvSt.hpp"
#include"CtvDll.hpp"
#include"_TUMObject.hpp"

_PzPtr _PzAll, _PzAllUMObject;
MatrRPtr _lMassMatrix;

EXTERN_C void cdecl InPzMem( integer _nsubs );
EXTERN_C void cdecl FreePzMem( integer _nsubs );
EXTERN_C void cdecl ParCalc( real_ _t, integer _isubs, VectRPtr _parmarr );
EXTERN_C void cdecl GetIdentifiers( integer _isubs, VectRPtr _parmarr );
EXTERN_C pointer cdecl GetParamStructure( integer _isubs );
/* implementation */

EXTERN_C void cdecl InPzMem( integer _nsubs )
 {
  integer i;
  _UMObjectVarPtr _;
  _PzAll = (_UMObjectVars**)UMGetMem( _nsubs*sizeof(pointer) );
  _PzAllUMObject = _PzAll;
  LocalSubsystemCount = _nsubs;
  for( i = 1; i <= _nsubs; i++ )
   {
    _PzAll[i-1] = (_UMObjectVars*)UMGetMem( sizeof(_UMObjectVars) );
    _PzAll[i-1]->_ = (real_*)UMGetMem( 521*sizeof(real_) );
    _ = _PzAll[i-1];
    _->_[67] = 2;
   }
 }

EXTERN_C void cdecl FreePzMem( integer _nsubs )
 {
  integer i, j;
  for( i = 1; i <= _nsubs; i++ )
   {
    UMFreeMem( _PzAll[i-1]->_ );
    UMFreeMem( _PzAll[i-1] );
   }
  UMFreeMem( _PzAll );
 }

EXTERN_C void cdecl ParCalc( real_ _t, integer _isubs, VectRPtr _parmarr )
 {
  _UMObjectVarPtr _;
  t = _t;
  _ = _PzAll[SubIndx[_isubs-1]-1];
  _->a = _parmarr[0];
 }

EXTERN_C void cdecl GetIdentifiers( integer _isubs, VectRPtr _parmarr )
 {
  _UMObjectVarPtr _;
  _ = _PzAll[SubIndx[_isubs-1]-1];
  _parmarr[0] = _->a;
 }

EXTERN_C pointer cdecl GetParamStructure( integer _isubs )
 {
  return _PzAll[SubIndx[_isubs-1]-1];
 }

/* end of file */
#endif
