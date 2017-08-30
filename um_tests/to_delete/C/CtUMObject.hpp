/* Header file CtUMObject */
#ifndef __CtUMObject_HPP__
#define __CtUMObject_HPP__

/* Constraint equations */
/* interface */

#include"CtvSt.hpp"
#include"CtvDll.hpp"

EXTERN_C void cdecl  ConstrCalc( pointer _Joint, VectRPtr _x, VectRPtr _v, integer _index, integer _isubs );
/* implementation */

#include"UMObjectC.hpp"
#include"_TUMObject.hpp"

EXTERN_C void cdecl ConstrCalc( pointer _Joint, VectRPtr _x, VectRPtr _v, integer _index, integer _isubs )
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
