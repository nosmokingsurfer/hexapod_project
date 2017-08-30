/* Library UmTask64 */
#include <stdlib.h>
#ifdef __BUILDER__
#include <windows.h>
int WINAPI DllMain(HINSTANCE hinstDLL, DWORD fwdreason, LPVOID lpvReserved) { return 1; }
#else
int DllMain() { return 1; }
#endif

#include"CtvDll.hpp"
#include"ClUMObject.hpp"
#include"_KUMObject.hpp"
#include"CtUMObject.hpp"
#include"EqUMObject.hpp"
#include"AlUMObject.hpp"
#include"UMObjectC.hpp"
#include"DGetVars.hpp"

EXTERN_C void cdecl DllInitData( DllConnRec &DllRec )
 {
  SubIndx = DllRec.SIndx;
  NSubSystems = DllRec.NSubsys;
  ParmArr = DllRec.PArr;
  UserVars = DllRec.UVarsPtr;
  GetVarsProc = *(DllRec.GetVarsProcPtr);
  _FrcJCalc = DllRec.FrcJCalc_;
  CommonData = DllRec.CommonDataSource;
  _bForce = DllRec.bForce;
  _bMoment = DllRec.bMoment;
  _bQuaternion = DllRec.bQuaternion;
  _eQuaternion = DllRec.eQuaternion;
  _Mass = DllRec.mass;
  _RCenter = DllRec.rcenter;
  _TInertia = DllRec.tinertia;
  _NAV = DllRec.__NAV;
  InitProcedures();
 }

EXTERN_C integer cdecl UMVer(  )
 {
  return 80;
 }

/* end of file */
