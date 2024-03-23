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
  #define CASADI_PREFIX(ID) nmpc_planner_cost_ext_cost_0_fun_jac_hess_ ## ID
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
#define casadi_s5 CASADI_PREFIX(s5)
#define casadi_s6 CASADI_PREFIX(s6)

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

static const casadi_int casadi_s0[8] = {4, 1, 0, 4, 0, 1, 2, 3};
static const casadi_int casadi_s1[6] = {2, 1, 0, 2, 0, 1};
static const casadi_int casadi_s2[3] = {0, 0, 0};
static const casadi_int casadi_s3[10] = {6, 1, 0, 6, 0, 1, 2, 3, 4, 5};
static const casadi_int casadi_s4[5] = {1, 1, 0, 1, 0};
static const casadi_int casadi_s5[13] = {6, 6, 0, 1, 2, 3, 4, 4, 4, 0, 1, 2, 3};
static const casadi_int casadi_s6[9] = {0, 6, 0, 0, 0, 0, 0, 0, 0};

/* nmpc_planner_cost_ext_cost_0_fun_jac_hess:(i0[4],i1[2],i2[],i3[6])->(o0,o1[6],o2[6x6,4nz],o3[],o4[0x6]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a2, a3, a4, a5, a6, a7, a8, a9;
  a0=120.;
  a1=arg[0]? arg[0][0] : 0;
  a2=arg[3]? arg[3][0] : 0;
  a3=(a1-a2);
  a3=(a0*a3);
  a1=(a1-a2);
  a2=(a3*a1);
  a4=100.;
  a5=arg[0]? arg[0][1] : 0;
  a6=arg[3]? arg[3][1] : 0;
  a7=(a5-a6);
  a7=(a4*a7);
  a6=(a5-a6);
  a8=(a7*a6);
  a2=(a2+a8);
  a8=30.;
  a9=arg[1]? arg[1][0] : 0;
  a10=(a8*a9);
  a11=(a10*a9);
  a12=800.;
  a13=arg[1]? arg[1][1] : 0;
  a14=(a12*a13);
  a15=(a14*a13);
  a11=(a11+a15);
  a2=(a2+a11);
  a11=200.;
  a15=arg[3]? arg[3][5] : 0;
  a16=(a5-a15);
  a16=(a11*a16);
  a5=(a5-a15);
  a15=(a16*a5);
  a2=(a2+a15);
  if (res[0]!=0) res[0][0]=a2;
  a8=(a8*a9);
  a10=(a10+a8);
  if (res[1]!=0) res[1][0]=a10;
  a12=(a12*a13);
  a14=(a14+a12);
  if (res[1]!=0) res[1][1]=a14;
  a0=(a0*a1);
  a3=(a3+a0);
  if (res[1]!=0) res[1][2]=a3;
  a11=(a11*a5);
  a16=(a16+a11);
  a16=(a16+a7);
  a4=(a4*a6);
  a16=(a16+a4);
  if (res[1]!=0) res[1][3]=a16;
  a16=0.;
  if (res[1]!=0) res[1][4]=a16;
  if (res[1]!=0) res[1][5]=a16;
  a16=60.;
  if (res[2]!=0) res[2][0]=a16;
  a16=1600.;
  if (res[2]!=0) res[2][1]=a16;
  a16=240.;
  if (res[2]!=0) res[2][2]=a16;
  a16=600.;
  if (res[2]!=0) res[2][3]=a16;
  return 0;
}

CASADI_SYMBOL_EXPORT int nmpc_planner_cost_ext_cost_0_fun_jac_hess(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int nmpc_planner_cost_ext_cost_0_fun_jac_hess_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int nmpc_planner_cost_ext_cost_0_fun_jac_hess_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void nmpc_planner_cost_ext_cost_0_fun_jac_hess_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int nmpc_planner_cost_ext_cost_0_fun_jac_hess_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void nmpc_planner_cost_ext_cost_0_fun_jac_hess_release(int mem) {
}

CASADI_SYMBOL_EXPORT void nmpc_planner_cost_ext_cost_0_fun_jac_hess_incref(void) {
}

CASADI_SYMBOL_EXPORT void nmpc_planner_cost_ext_cost_0_fun_jac_hess_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int nmpc_planner_cost_ext_cost_0_fun_jac_hess_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int nmpc_planner_cost_ext_cost_0_fun_jac_hess_n_out(void) { return 5;}

CASADI_SYMBOL_EXPORT casadi_real nmpc_planner_cost_ext_cost_0_fun_jac_hess_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* nmpc_planner_cost_ext_cost_0_fun_jac_hess_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* nmpc_planner_cost_ext_cost_0_fun_jac_hess_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    case 3: return "o3";
    case 4: return "o4";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* nmpc_planner_cost_ext_cost_0_fun_jac_hess_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    case 3: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* nmpc_planner_cost_ext_cost_0_fun_jac_hess_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s4;
    case 1: return casadi_s3;
    case 2: return casadi_s5;
    case 3: return casadi_s2;
    case 4: return casadi_s6;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int nmpc_planner_cost_ext_cost_0_fun_jac_hess_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 4;
  if (sz_res) *sz_res = 5;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
