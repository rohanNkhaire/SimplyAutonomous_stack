/* This file was automatically generated by CasADi 3.6.4.
 *  It consists of: 
 *   1) content generated by CasADi runtime: not copyrighted
 *   2) template code copied from CasADi source: permissively licensed (MIT-0)
 *   3) user code: owned by the user
 *
 */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) nmpc_planner_steer_expl_vde_forw_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_s4 CASADI_PREFIX(s4)
#define casadi_sq CASADI_PREFIX(sq)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

casadi_real casadi_sq(casadi_real x) { return x*x;}

static const casadi_int casadi_s0[9] = {5, 1, 0, 5, 0, 1, 2, 3, 4};
static const casadi_int casadi_s1[33] = {5, 5, 0, 5, 10, 15, 20, 25, 0, 1, 2, 3, 4, 0, 1, 2, 3, 4, 0, 1, 2, 3, 4, 0, 1, 2, 3, 4, 0, 1, 2, 3, 4};
static const casadi_int casadi_s2[15] = {5, 2, 0, 5, 10, 0, 1, 2, 3, 4, 0, 1, 2, 3, 4};
static const casadi_int casadi_s3[6] = {2, 1, 0, 2, 0, 1};
static const casadi_int casadi_s4[3] = {0, 0, 0};

/* nmpc_planner_steer_expl_vde_forw:(i0[5],i1[5x5],i2[5x2],i3[2],i4[])->(o0[5],o1[5x5],o2[5x2]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a3, a4, a5, a6, a7, a8, a9;
  a0=arg[0]? arg[0][2] : 0;
  a1=arg[0]? arg[0][3] : 0;
  a2=arg[0]? arg[0][4] : 0;
  a3=(a1+a2);
  a4=cos(a3);
  a5=(a0*a4);
  if (res[0]!=0) res[0][0]=a5;
  a1=(a1+a2);
  a5=sin(a1);
  a6=(a0*a5);
  if (res[0]!=0) res[0][1]=a6;
  a6=arg[3]? arg[3][0] : 0;
  if (res[0]!=0) res[0][2]=a6;
  a6=1.4375000000000000e+00;
  a6=(a0/a6);
  a7=atan(a2);
  a8=sin(a7);
  a9=(a6*a8);
  if (res[0]!=0) res[0][3]=a9;
  a9=5.0000000000000000e-01;
  a10=arg[3]? arg[3][1] : 0;
  a11=tan(a10);
  a11=(a9*a11);
  if (res[0]!=0) res[0][4]=a11;
  a11=arg[1]? arg[1][2] : 0;
  a12=(a4*a11);
  a13=sin(a3);
  a14=arg[1]? arg[1][3] : 0;
  a15=arg[1]? arg[1][4] : 0;
  a16=(a14+a15);
  a16=(a13*a16);
  a16=(a0*a16);
  a12=(a12-a16);
  if (res[1]!=0) res[1][0]=a12;
  a12=(a5*a11);
  a16=cos(a1);
  a14=(a14+a15);
  a14=(a16*a14);
  a14=(a0*a14);
  a12=(a12+a14);
  if (res[1]!=0) res[1][1]=a12;
  a12=0.;
  if (res[1]!=0) res[1][2]=a12;
  a14=6.9565217391304346e-01;
  a11=(a14*a11);
  a11=(a8*a11);
  a17=cos(a7);
  a18=1.;
  a19=casadi_sq(a2);
  a19=(a18+a19);
  a15=(a15/a19);
  a15=(a17*a15);
  a15=(a6*a15);
  a11=(a11+a15);
  if (res[1]!=0) res[1][3]=a11;
  if (res[1]!=0) res[1][4]=a12;
  a11=arg[1]? arg[1][7] : 0;
  a15=(a4*a11);
  a20=arg[1]? arg[1][8] : 0;
  a21=arg[1]? arg[1][9] : 0;
  a22=(a20+a21);
  a22=(a13*a22);
  a22=(a0*a22);
  a15=(a15-a22);
  if (res[1]!=0) res[1][5]=a15;
  a15=(a5*a11);
  a20=(a20+a21);
  a20=(a16*a20);
  a20=(a0*a20);
  a15=(a15+a20);
  if (res[1]!=0) res[1][6]=a15;
  if (res[1]!=0) res[1][7]=a12;
  a11=(a14*a11);
  a11=(a8*a11);
  a21=(a21/a19);
  a21=(a17*a21);
  a21=(a6*a21);
  a11=(a11+a21);
  if (res[1]!=0) res[1][8]=a11;
  if (res[1]!=0) res[1][9]=a12;
  a11=arg[1]? arg[1][12] : 0;
  a21=(a4*a11);
  a15=arg[1]? arg[1][13] : 0;
  a20=arg[1]? arg[1][14] : 0;
  a22=(a15+a20);
  a22=(a13*a22);
  a22=(a0*a22);
  a21=(a21-a22);
  if (res[1]!=0) res[1][10]=a21;
  a21=(a5*a11);
  a15=(a15+a20);
  a15=(a16*a15);
  a15=(a0*a15);
  a21=(a21+a15);
  if (res[1]!=0) res[1][11]=a21;
  if (res[1]!=0) res[1][12]=a12;
  a11=(a14*a11);
  a11=(a8*a11);
  a20=(a20/a19);
  a20=(a17*a20);
  a20=(a6*a20);
  a11=(a11+a20);
  if (res[1]!=0) res[1][13]=a11;
  if (res[1]!=0) res[1][14]=a12;
  a11=arg[1]? arg[1][17] : 0;
  a20=(a4*a11);
  a21=arg[1]? arg[1][18] : 0;
  a15=arg[1]? arg[1][19] : 0;
  a22=(a21+a15);
  a22=(a13*a22);
  a22=(a0*a22);
  a20=(a20-a22);
  if (res[1]!=0) res[1][15]=a20;
  a20=(a5*a11);
  a21=(a21+a15);
  a21=(a16*a21);
  a21=(a0*a21);
  a20=(a20+a21);
  if (res[1]!=0) res[1][16]=a20;
  if (res[1]!=0) res[1][17]=a12;
  a11=(a14*a11);
  a11=(a8*a11);
  a15=(a15/a19);
  a15=(a17*a15);
  a15=(a6*a15);
  a11=(a11+a15);
  if (res[1]!=0) res[1][18]=a11;
  if (res[1]!=0) res[1][19]=a12;
  a11=arg[1]? arg[1][22] : 0;
  a15=(a4*a11);
  a20=arg[1]? arg[1][23] : 0;
  a21=arg[1]? arg[1][24] : 0;
  a22=(a20+a21);
  a13=(a13*a22);
  a13=(a0*a13);
  a15=(a15-a13);
  if (res[1]!=0) res[1][20]=a15;
  a15=(a5*a11);
  a20=(a20+a21);
  a16=(a16*a20);
  a16=(a0*a16);
  a15=(a15+a16);
  if (res[1]!=0) res[1][21]=a15;
  if (res[1]!=0) res[1][22]=a12;
  a11=(a14*a11);
  a11=(a8*a11);
  a21=(a21/a19);
  a17=(a17*a21);
  a17=(a6*a17);
  a11=(a11+a17);
  if (res[1]!=0) res[1][23]=a11;
  if (res[1]!=0) res[1][24]=a12;
  a11=arg[2]? arg[2][2] : 0;
  a17=(a4*a11);
  a3=sin(a3);
  a21=arg[2]? arg[2][3] : 0;
  a19=arg[2]? arg[2][4] : 0;
  a15=(a21+a19);
  a15=(a3*a15);
  a15=(a0*a15);
  a17=(a17-a15);
  if (res[2]!=0) res[2][0]=a17;
  a17=(a5*a11);
  a1=cos(a1);
  a21=(a21+a19);
  a21=(a1*a21);
  a21=(a0*a21);
  a17=(a17+a21);
  if (res[2]!=0) res[2][1]=a17;
  if (res[2]!=0) res[2][2]=a18;
  a11=(a14*a11);
  a11=(a8*a11);
  a7=cos(a7);
  a2=casadi_sq(a2);
  a18=(a18+a2);
  a19=(a19/a18);
  a19=(a7*a19);
  a19=(a6*a19);
  a11=(a11+a19);
  if (res[2]!=0) res[2][3]=a11;
  if (res[2]!=0) res[2][4]=a12;
  a11=arg[2]? arg[2][7] : 0;
  a4=(a4*a11);
  a19=arg[2]? arg[2][8] : 0;
  a2=arg[2]? arg[2][9] : 0;
  a17=(a19+a2);
  a3=(a3*a17);
  a3=(a0*a3);
  a4=(a4-a3);
  if (res[2]!=0) res[2][5]=a4;
  a5=(a5*a11);
  a19=(a19+a2);
  a1=(a1*a19);
  a0=(a0*a1);
  a5=(a5+a0);
  if (res[2]!=0) res[2][6]=a5;
  if (res[2]!=0) res[2][7]=a12;
  a14=(a14*a11);
  a8=(a8*a14);
  a2=(a2/a18);
  a7=(a7*a2);
  a6=(a6*a7);
  a8=(a8+a6);
  if (res[2]!=0) res[2][8]=a8;
  a10=cos(a10);
  a10=casadi_sq(a10);
  a9=(a9/a10);
  if (res[2]!=0) res[2][9]=a9;
  return 0;
}

CASADI_SYMBOL_EXPORT int nmpc_planner_steer_expl_vde_forw(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int nmpc_planner_steer_expl_vde_forw_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int nmpc_planner_steer_expl_vde_forw_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void nmpc_planner_steer_expl_vde_forw_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int nmpc_planner_steer_expl_vde_forw_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void nmpc_planner_steer_expl_vde_forw_release(int mem) {
}

CASADI_SYMBOL_EXPORT void nmpc_planner_steer_expl_vde_forw_incref(void) {
}

CASADI_SYMBOL_EXPORT void nmpc_planner_steer_expl_vde_forw_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int nmpc_planner_steer_expl_vde_forw_n_in(void) { return 5;}

CASADI_SYMBOL_EXPORT casadi_int nmpc_planner_steer_expl_vde_forw_n_out(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_real nmpc_planner_steer_expl_vde_forw_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* nmpc_planner_steer_expl_vde_forw_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    case 4: return "i4";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* nmpc_planner_steer_expl_vde_forw_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* nmpc_planner_steer_expl_vde_forw_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    case 3: return casadi_s3;
    case 4: return casadi_s4;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* nmpc_planner_steer_expl_vde_forw_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int nmpc_planner_steer_expl_vde_forw_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 5;
  if (sz_res) *sz_res = 3;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif