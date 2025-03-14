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
  #define CASADI_PREFIX(ID) kin6_impl_dae_jac_x_xdot_u_z_ ## ID
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
#define casadi_s7 CASADI_PREFIX(s7)
#define casadi_s8 CASADI_PREFIX(s8)
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

static const casadi_int casadi_s0[15] = {11, 1, 0, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
static const casadi_int casadi_s1[9] = {5, 1, 0, 5, 0, 1, 2, 3, 4};
static const casadi_int casadi_s2[4] = {0, 1, 0, 0};
static const casadi_int casadi_s3[3] = {0, 0, 0};
static const casadi_int casadi_s4[14] = {10, 1, 0, 10, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
static const casadi_int casadi_s5[45] = {11, 11, 0, 0, 0, 2, 7, 10, 11, 15, 19, 23, 27, 31, 0, 1, 0, 1, 3, 4, 5, 0, 1, 3, 2, 3, 4, 5, 6, 3, 4, 5, 7, 3, 4, 5, 8, 3, 4, 5, 9, 3, 4, 5, 10};
static const casadi_int casadi_s6[25] = {11, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
static const casadi_int casadi_s7[16] = {11, 5, 0, 4, 5, 6, 7, 8, 3, 4, 5, 6, 7, 8, 9, 10};
static const casadi_int casadi_s8[3] = {11, 0, 0};

/* kin6_impl_dae_jac_x_xdot_u_z:(i0[11],i1[11],i2[5],i3[0],i4[],i5[10])->(o0[11x11,31nz],o1[11x11,11nz],o2[11x5,8nz],o3[11x0]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a2, a3, a4, a5, a6, a7, a8, a9;
  a0=arg[0]? arg[0][3] : 0;
  a1=arg[0]? arg[0][2] : 0;
  a2=sin(a1);
  a3=(a0*a2);
  a4=arg[0]? arg[0][4] : 0;
  a1=cos(a1);
  a5=(a4*a1);
  a3=(a3+a5);
  if (res[0]!=0) res[0][0]=a3;
  a3=(a0*a1);
  a5=(a4*a2);
  a3=(a3-a5);
  a3=(-a3);
  if (res[0]!=0) res[0][1]=a3;
  a1=(-a1);
  if (res[0]!=0) res[0][2]=a1;
  a3=(-a2);
  if (res[0]!=0) res[0][3]=a3;
  a3=arg[5]? arg[5][2] : 0;
  a5=arg[5]? arg[5][3] : 0;
  a5=(a3+a5);
  a5=(a3/a5);
  a6=arg[0]? arg[0][6] : 0;
  a7=tan(a6);
  a7=(a5*a7);
  a8=atan(a7);
  a9=cos(a8);
  a10=10.;
  a11=(a10*a0);
  a11=tanh(a11);
  a12=arg[5]? arg[5][6] : 0;
  a13=arg[5]? arg[5][7] : 0;
  a14=(a0+a0);
  a14=(a13*a14);
  a14=(a12+a14);
  a14=(a11*a14);
  a15=arg[5]? arg[5][5] : 0;
  a12=(a12*a0);
  a15=(a15+a12);
  a12=casadi_sq(a0);
  a13=(a13*a12);
  a15=(a15+a13);
  a13=1.;
  a12=casadi_sq(a11);
  a12=(a13-a12);
  a10=(a10*a12);
  a10=(a15*a10);
  a14=(a14+a10);
  a14=(a9*a14);
  a10=arg[5]? arg[5][0] : 0;
  a14=(a14/a10);
  a12=(a9*a14);
  if (res[0]!=0) res[0][4]=a12;
  a12=arg[2]? arg[2][0] : 0;
  a12=(a12-a6);
  a16=arg[5]? arg[5][9] : 0;
  a12=(a12/a16);
  a12=(a5*a12);
  a17=sin(a8);
  a14=(a17*a14);
  a14=(a12-a14);
  a18=(-a14);
  if (res[0]!=0) res[0][5]=a18;
  a14=(a14/a3);
  a14=(-a14);
  if (res[0]!=0) res[0][6]=a14;
  if (res[0]!=0) res[0][7]=a2;
  if (res[0]!=0) res[0][8]=a1;
  if (res[0]!=0) res[0][9]=a12;
  a12=-1.;
  if (res[0]!=0) res[0][10]=a12;
  a12=(a5/a16);
  a4=(a4*a12);
  a1=5.0000000000000000e-01;
  a2=arg[5]? arg[5][4] : 0;
  a14=arg[0]? arg[0][7] : 0;
  a18=arg[0]? arg[0][8] : 0;
  a14=(a14+a18);
  a18=arg[0]? arg[0][9] : 0;
  a14=(a14+a18);
  a18=arg[0]? arg[0][10] : 0;
  a14=(a14+a18);
  a14=(a2*a14);
  a14=(a1*a14);
  a15=(a15*a11);
  a15=(a14-a15);
  a11=cos(a6);
  a11=casadi_sq(a11);
  a5=(a5/a11);
  a7=casadi_sq(a7);
  a7=(a13+a7);
  a5=(a5/a7);
  a7=(a17*a5);
  a11=(a15*a7);
  a6=(a6-a8);
  a8=sin(a6);
  a18=(a13-a5);
  a8=(a8*a18);
  a8=(a14*a8);
  a11=(a11+a8);
  a11=(a11/a10);
  a8=(a9*a11);
  a15=(a9*a15);
  a6=cos(a6);
  a14=(a6*a14);
  a15=(a15+a14);
  a15=(a15/a10);
  a7=(a15*a7);
  a8=(a8+a7);
  a8=(a4-a8);
  a8=(-a8);
  if (res[0]!=0) res[0][11]=a8;
  a5=(a9*a5);
  a15=(a15*a5);
  a11=(a17*a11);
  a15=(a15-a11);
  a0=(a0*a12);
  a15=(a15-a0);
  a12=(-a15);
  if (res[0]!=0) res[0][12]=a12;
  a15=(a15/a3);
  a15=(-a15);
  if (res[0]!=0) res[0][13]=a15;
  a16=(1./a16);
  if (res[0]!=0) res[0][14]=a16;
  a1=(a1*a2);
  a2=(a9*a1);
  a6=(a6*a1);
  a2=(a2+a6);
  a2=(a2/a10);
  a9=(a9*a2);
  a9=(-a9);
  if (res[0]!=0) res[0][15]=a9;
  a17=(a17*a2);
  a2=(-a17);
  if (res[0]!=0) res[0][16]=a2;
  a17=(a17/a3);
  a17=(-a17);
  if (res[0]!=0) res[0][17]=a17;
  a10=arg[5]? arg[5][8] : 0;
  a10=(1./a10);
  if (res[0]!=0) res[0][18]=a10;
  if (res[0]!=0) res[0][19]=a9;
  if (res[0]!=0) res[0][20]=a2;
  if (res[0]!=0) res[0][21]=a17;
  if (res[0]!=0) res[0][22]=a10;
  if (res[0]!=0) res[0][23]=a9;
  if (res[0]!=0) res[0][24]=a2;
  if (res[0]!=0) res[0][25]=a17;
  if (res[0]!=0) res[0][26]=a10;
  if (res[0]!=0) res[0][27]=a9;
  if (res[0]!=0) res[0][28]=a2;
  if (res[0]!=0) res[0][29]=a17;
  if (res[0]!=0) res[0][30]=a10;
  if (res[1]!=0) res[1][0]=a13;
  if (res[1]!=0) res[1][1]=a13;
  if (res[1]!=0) res[1][2]=a13;
  if (res[1]!=0) res[1][3]=a13;
  if (res[1]!=0) res[1][4]=a13;
  if (res[1]!=0) res[1][5]=a13;
  if (res[1]!=0) res[1][6]=a13;
  if (res[1]!=0) res[1][7]=a13;
  if (res[1]!=0) res[1][8]=a13;
  if (res[1]!=0) res[1][9]=a13;
  if (res[1]!=0) res[1][10]=a13;
  if (res[2]!=0) res[2][0]=a4;
  a4=(-a0);
  if (res[2]!=0) res[2][1]=a4;
  a0=(a0/a3);
  a0=(-a0);
  if (res[2]!=0) res[2][2]=a0;
  a16=(-a16);
  if (res[2]!=0) res[2][3]=a16;
  a10=(-a10);
  if (res[2]!=0) res[2][4]=a10;
  if (res[2]!=0) res[2][5]=a10;
  if (res[2]!=0) res[2][6]=a10;
  if (res[2]!=0) res[2][7]=a10;
  return 0;
}

CASADI_SYMBOL_EXPORT int kin6_impl_dae_jac_x_xdot_u_z(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int kin6_impl_dae_jac_x_xdot_u_z_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int kin6_impl_dae_jac_x_xdot_u_z_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void kin6_impl_dae_jac_x_xdot_u_z_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int kin6_impl_dae_jac_x_xdot_u_z_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void kin6_impl_dae_jac_x_xdot_u_z_release(int mem) {
}

CASADI_SYMBOL_EXPORT void kin6_impl_dae_jac_x_xdot_u_z_incref(void) {
}

CASADI_SYMBOL_EXPORT void kin6_impl_dae_jac_x_xdot_u_z_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int kin6_impl_dae_jac_x_xdot_u_z_n_in(void) { return 6;}

CASADI_SYMBOL_EXPORT casadi_int kin6_impl_dae_jac_x_xdot_u_z_n_out(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_real kin6_impl_dae_jac_x_xdot_u_z_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* kin6_impl_dae_jac_x_xdot_u_z_name_in(casadi_int i) {
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

CASADI_SYMBOL_EXPORT const char* kin6_impl_dae_jac_x_xdot_u_z_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    case 3: return "o3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* kin6_impl_dae_jac_x_xdot_u_z_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s0;
    case 2: return casadi_s1;
    case 3: return casadi_s2;
    case 4: return casadi_s3;
    case 5: return casadi_s4;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* kin6_impl_dae_jac_x_xdot_u_z_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s5;
    case 1: return casadi_s6;
    case 2: return casadi_s7;
    case 3: return casadi_s8;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int kin6_impl_dae_jac_x_xdot_u_z_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 6;
  if (sz_res) *sz_res = 4;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

CASADI_SYMBOL_EXPORT int kin6_impl_dae_jac_x_xdot_u_z_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 6*sizeof(const casadi_real*);
  if (sz_res) *sz_res = 4*sizeof(casadi_real*);
  if (sz_iw) *sz_iw = 0*sizeof(casadi_int);
  if (sz_w) *sz_w = 0*sizeof(casadi_real);
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
