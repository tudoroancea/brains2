/* This file was automatically generated by CasADi 3.6.7.
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
  #define CASADI_PREFIX(ID) dyn6_impl_dae_fun_ ## ID
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

static const casadi_int casadi_s0[15] = {11, 1, 0, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
static const casadi_int casadi_s1[9] = {5, 1, 0, 5, 0, 1, 2, 3, 4};
static const casadi_int casadi_s2[3] = {0, 0, 0};
static const casadi_int casadi_s3[21] = {17, 1, 0, 17, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};

/* dyn6_impl_dae_fun:(i0[11],i1[11],i2[5],i3[],i4[],i5[17])->(o0[11]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a4, a5, a6, a7, a8, a9;
  a0=arg[1]? arg[1][0] : 0;
  a1=arg[0]? arg[0][3] : 0;
  a2=arg[0]? arg[0][2] : 0;
  a3=cos(a2);
  a4=(a1*a3);
  a5=arg[0]? arg[0][4] : 0;
  a2=sin(a2);
  a6=(a5*a2);
  a4=(a4-a6);
  a0=(a0-a4);
  if (res[0]!=0) res[0][0]=a0;
  a0=arg[1]? arg[1][1] : 0;
  a2=(a1*a2);
  a3=(a5*a3);
  a2=(a2+a3);
  a0=(a0-a2);
  if (res[0]!=0) res[0][1]=a0;
  a0=arg[1]? arg[1][2] : 0;
  a2=arg[0]? arg[0][5] : 0;
  a0=(a0-a2);
  if (res[0]!=0) res[0][2]=a0;
  a0=arg[5]? arg[5][0] : 0;
  a3=arg[1]? arg[1][3] : 0;
  a4=(a5*a2);
  a3=(a3-a4);
  a4=(a0*a3);
  a6=arg[5]? arg[5][4] : 0;
  a7=arg[0]? arg[0][8] : 0;
  a8=(a6*a7);
  a9=arg[0]? arg[0][7] : 0;
  a10=(a6*a9);
  a11=(a8+a10);
  a12=arg[0]? arg[0][6] : 0;
  a13=cos(a12);
  a14=(a11*a13);
  a15=5.0000000000000000e-01;
  a16=arg[5]? arg[5][3] : 0;
  a17=(a15*a16);
  a18=arg[5]? arg[5][2] : 0;
  a19=(a18+a16);
  a17=(a17/a19);
  a20=9.8100000000000005e+00;
  a20=(a20*a0);
  a21=arg[5]? arg[5][12] : 0;
  a21=(a15*a21);
  a21=(a21*a1);
  a21=(a21*a1);
  a20=(a20+a21);
  a17=(a17*a20);
  a21=(a15*a0);
  a3=(a21*a3);
  a22=arg[5]? arg[5][10] : 0;
  a3=(a3*a22);
  a3=(a3/a19);
  a17=(a17-a3);
  a23=arg[1]? arg[1][4] : 0;
  a24=(a1*a2);
  a23=(a23+a24);
  a21=(a21*a23);
  a21=(a21*a22);
  a22=arg[5]? arg[5][11] : 0;
  a21=(a21/a22);
  a24=(a17+a21);
  a25=arg[5]? arg[5][15] : 0;
  a26=arg[5]? arg[5][14] : 0;
  a27=arg[5]? arg[5][13] : 0;
  a28=(a16*a2);
  a28=(a5+a28);
  a29=(a15*a22);
  a29=(a29*a2);
  a30=(a1+a29);
  a31=atan2(a28,a30);
  a31=(a31-a12);
  a31=(a27*a31);
  a32=arg[5]? arg[5][16] : 0;
  a33=atan(a31);
  a33=(a31-a33);
  a33=(a32*a33);
  a31=(a31-a33);
  a31=atan(a31);
  a31=(a26*a31);
  a31=sin(a31);
  a31=(a25*a31);
  a24=(a24*a31);
  a17=(a17-a21);
  a29=(a1-a29);
  a28=atan2(a28,a29);
  a28=(a28-a12);
  a28=(a27*a28);
  a31=atan(a28);
  a31=(a28-a31);
  a31=(a32*a31);
  a28=(a28-a31);
  a28=atan(a28);
  a28=(a26*a28);
  a28=sin(a28);
  a28=(a25*a28);
  a17=(a17*a28);
  a28=(a24+a17);
  a31=sin(a12);
  a33=(a28*a31);
  a14=(a14+a33);
  a33=arg[0]? arg[0][10] : 0;
  a34=(a6*a33);
  a14=(a14+a34);
  a35=arg[0]? arg[0][9] : 0;
  a6=(a6*a35);
  a14=(a14+a6);
  a36=arg[5]? arg[5][5] : 0;
  a37=arg[5]? arg[5][6] : 0;
  a37=(a37*a1);
  a36=(a36+a37);
  a37=arg[5]? arg[5][7] : 0;
  a37=(a37*a1);
  a37=(a37*a1);
  a36=(a36+a37);
  a37=10.;
  a37=(a37*a1);
  a37=tanh(a37);
  a36=(a36*a37);
  a14=(a14-a36);
  a4=(a4-a14);
  if (res[0]!=0) res[0][3]=a4;
  a0=(a0*a23);
  a11=(a11*a31);
  a28=(a28*a13);
  a11=(a11-a28);
  a15=(a15*a18);
  a15=(a15/a19);
  a15=(a15*a20);
  a15=(a15+a3);
  a3=(a15+a21);
  a2=(a18*a2);
  a5=(a5-a2);
  a30=atan2(a5,a30);
  a30=(a27*a30);
  a2=atan(a30);
  a2=(a30-a2);
  a2=(a32*a2);
  a30=(a30-a2);
  a30=atan(a30);
  a30=(a26*a30);
  a30=sin(a30);
  a30=(a25*a30);
  a3=(a3*a30);
  a11=(a11-a3);
  a15=(a15-a21);
  a5=atan2(a5,a29);
  a27=(a27*a5);
  a5=atan(a27);
  a5=(a27-a5);
  a32=(a32*a5);
  a27=(a27-a32);
  a27=atan(a27);
  a26=(a26*a27);
  a26=sin(a26);
  a25=(a25*a26);
  a15=(a15*a25);
  a11=(a11-a15);
  a0=(a0-a11);
  if (res[0]!=0) res[0][4]=a0;
  a0=arg[5]? arg[5][1] : 0;
  a11=arg[1]? arg[1][5] : 0;
  a0=(a0*a11);
  a11=(a8*a13);
  a25=(a24*a31);
  a11=(a11+a25);
  a11=(a11*a22);
  a25=2.;
  a11=(a11/a25);
  a8=(a8*a31);
  a24=(a24*a13);
  a8=(a8-a24);
  a8=(a8*a16);
  a11=(a11+a8);
  a8=(a10*a13);
  a24=(a17*a31);
  a8=(a8+a24);
  a8=(a8*a22);
  a8=(a8/a25);
  a11=(a11-a8);
  a10=(a10*a31);
  a17=(a17*a13);
  a10=(a10-a17);
  a10=(a10*a16);
  a11=(a11+a10);
  a34=(a34*a22);
  a34=(a34/a25);
  a11=(a11+a34);
  a3=(a3*a18);
  a11=(a11+a3);
  a6=(a6*a22);
  a6=(a6/a25);
  a11=(a11-a6);
  a15=(a15*a18);
  a11=(a11+a15);
  a0=(a0-a11);
  if (res[0]!=0) res[0][5]=a0;
  a0=arg[1]? arg[1][6] : 0;
  a11=arg[2]? arg[2][0] : 0;
  a11=(a11-a12);
  a12=arg[5]? arg[5][9] : 0;
  a11=(a11/a12);
  a0=(a0-a11);
  if (res[0]!=0) res[0][6]=a0;
  a0=arg[1]? arg[1][7] : 0;
  a11=arg[2]? arg[2][1] : 0;
  a11=(a11-a9);
  a9=arg[5]? arg[5][8] : 0;
  a11=(a11/a9);
  a0=(a0-a11);
  if (res[0]!=0) res[0][7]=a0;
  a0=arg[1]? arg[1][8] : 0;
  a11=arg[2]? arg[2][2] : 0;
  a11=(a11-a7);
  a11=(a11/a9);
  a0=(a0-a11);
  if (res[0]!=0) res[0][8]=a0;
  a0=arg[1]? arg[1][9] : 0;
  a11=arg[2]? arg[2][3] : 0;
  a11=(a11-a35);
  a11=(a11/a9);
  a0=(a0-a11);
  if (res[0]!=0) res[0][9]=a0;
  a0=arg[1]? arg[1][10] : 0;
  a11=arg[2]? arg[2][4] : 0;
  a11=(a11-a33);
  a11=(a11/a9);
  a0=(a0-a11);
  if (res[0]!=0) res[0][10]=a0;
  return 0;
}

CASADI_SYMBOL_EXPORT int dyn6_impl_dae_fun(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int dyn6_impl_dae_fun_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int dyn6_impl_dae_fun_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void dyn6_impl_dae_fun_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int dyn6_impl_dae_fun_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void dyn6_impl_dae_fun_release(int mem) {
}

CASADI_SYMBOL_EXPORT void dyn6_impl_dae_fun_incref(void) {
}

CASADI_SYMBOL_EXPORT void dyn6_impl_dae_fun_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int dyn6_impl_dae_fun_n_in(void) { return 6;}

CASADI_SYMBOL_EXPORT casadi_int dyn6_impl_dae_fun_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real dyn6_impl_dae_fun_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* dyn6_impl_dae_fun_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    case 4: return "i4";
    case 5: return "i5";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* dyn6_impl_dae_fun_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* dyn6_impl_dae_fun_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s0;
    case 2: return casadi_s1;
    case 3: return casadi_s2;
    case 4: return casadi_s2;
    case 5: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* dyn6_impl_dae_fun_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int dyn6_impl_dae_fun_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 6;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

CASADI_SYMBOL_EXPORT int dyn6_impl_dae_fun_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 6*sizeof(const casadi_real*);
  if (sz_res) *sz_res = 1*sizeof(casadi_real*);
  if (sz_iw) *sz_iw = 0*sizeof(casadi_int);
  if (sz_w) *sz_w = 0*sizeof(casadi_real);
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
