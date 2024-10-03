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
  #define CASADI_PREFIX(ID) dyn6_impl_dae_fun_jac_x_xdot_u_ ## ID
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
static const casadi_int casadi_s2[3] = {0, 0, 0};
static const casadi_int casadi_s3[21] = {17, 1, 0, 17, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
static const casadi_int casadi_s4[48] = {11, 11, 0, 0, 0, 2, 7, 12, 16, 20, 24, 28, 31, 34, 0, 1, 0, 1, 3, 4, 5, 0, 1, 3, 4, 5, 2, 3, 4, 5, 3, 4, 5, 6, 3, 4, 5, 7, 3, 4, 5, 8, 3, 5, 9, 3, 5, 10};
static const casadi_int casadi_s5[29] = {11, 11, 0, 1, 2, 3, 6, 9, 10, 11, 12, 13, 14, 15, 0, 1, 2, 3, 4, 5, 3, 4, 5, 5, 6, 7, 8, 9, 10};
static const casadi_int casadi_s6[13] = {11, 5, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};

/* dyn6_impl_dae_fun_jac_x_xdot_u:(i0[11],i1[11],i2[5],i3[],i4[],i5[17])->(o0[11],o1[11x11,34nz],o2[11x11,15nz],o3[11x5,5nz]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a5, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a6, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a7, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a8, a80, a81, a82, a9;
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
  a6=(a1*a2);
  a7=(a5*a3);
  a6=(a6+a7);
  a0=(a0-a6);
  if (res[0]!=0) res[0][1]=a0;
  a0=arg[1]? arg[1][2] : 0;
  a7=arg[0]? arg[0][5] : 0;
  a0=(a0-a7);
  if (res[0]!=0) res[0][2]=a0;
  a0=arg[5]? arg[5][0] : 0;
  a8=arg[1]? arg[1][3] : 0;
  a9=(a5*a7);
  a8=(a8-a9);
  a8=(a0*a8);
  a9=arg[5]? arg[5][4] : 0;
  a10=arg[0]? arg[0][8] : 0;
  a11=(a9*a10);
  a12=arg[0]? arg[0][7] : 0;
  a13=(a9*a12);
  a14=(a11+a13);
  a15=arg[0]? arg[0][6] : 0;
  a16=cos(a15);
  a17=(a14*a16);
  a18=-5.0000000000000000e-01;
  a19=5.0000000000000000e-01;
  a20=arg[5]? arg[5][3] : 0;
  a21=(a19*a20);
  a22=arg[5]? arg[5][2] : 0;
  a23=(a22+a20);
  a21=(a21/a23);
  a24=9.8100000000000005e+00;
  a24=(a24*a0);
  a25=arg[5]? arg[5][12] : 0;
  a26=(a25*a1);
  a27=(a26*a1);
  a24=(a24+a27);
  a27=(a21*a24);
  a28=arg[5]? arg[5][10] : 0;
  a29=(a8*a28);
  a29=(a29/a23);
  a27=(a27-a29);
  a30=arg[1]? arg[1][4] : 0;
  a31=(a1*a7);
  a30=(a30+a31);
  a30=(a0*a30);
  a31=(a30*a28);
  a32=arg[5]? arg[5][11] : 0;
  a31=(a31/a32);
  a33=(a27-a31);
  a33=(a18*a33);
  a34=arg[5]? arg[5][15] : 0;
  a35=arg[5]? arg[5][14] : 0;
  a36=arg[5]? arg[5][13] : 0;
  a37=(a20*a7);
  a37=(a5+a37);
  a38=(a19*a32);
  a39=(a38*a7);
  a40=(a1+a39);
  a41=atan2(a37,a40);
  a41=(a41-a15);
  a41=(a36*a41);
  a42=arg[5]? arg[5][16] : 0;
  a43=atan(a41);
  a43=(a41-a43);
  a43=(a42*a43);
  a43=(a41-a43);
  a44=atan(a43);
  a44=(a35*a44);
  a45=sin(a44);
  a45=(a34*a45);
  a46=(a33*a45);
  a27=(a27+a31);
  a27=(a18*a27);
  a39=(a1-a39);
  a47=atan2(a37,a39);
  a47=(a47-a15);
  a47=(a36*a47);
  a48=atan(a47);
  a48=(a47-a48);
  a48=(a42*a48);
  a48=(a47-a48);
  a49=atan(a48);
  a49=(a35*a49);
  a50=sin(a49);
  a50=(a34*a50);
  a51=(a27*a50);
  a52=(a46+a51);
  a53=sin(a15);
  a54=(a52*a53);
  a55=(a17-a54);
  a56=arg[0]? arg[0][10] : 0;
  a57=(a9*a56);
  a55=(a55+a57);
  a58=arg[0]? arg[0][9] : 0;
  a59=(a9*a58);
  a55=(a55+a59);
  a60=arg[5]? arg[5][5] : 0;
  a61=arg[5]? arg[5][6] : 0;
  a62=(a61*a1);
  a60=(a60+a62);
  a62=arg[5]? arg[5][7] : 0;
  a63=(a62*a1);
  a64=(a63*a1);
  a60=(a60+a64);
  a64=10.;
  a65=(a64*a1);
  a65=tanh(a65);
  a66=(a60*a65);
  a55=(a55-a66);
  a8=(a8-a55);
  if (res[0]!=0) res[0][3]=a8;
  a14=(a14*a53);
  a52=(a52*a16);
  a8=(a14+a52);
  a55=(a19*a22);
  a55=(a55/a23);
  a24=(a55*a24);
  a24=(a24+a29);
  a29=(a24-a31);
  a29=(a18*a29);
  a66=(a22*a7);
  a66=(a5-a66);
  a67=atan2(a66,a40);
  a67=(a36*a67);
  a68=atan(a67);
  a68=(a67-a68);
  a68=(a42*a68);
  a68=(a67-a68);
  a69=atan(a68);
  a69=(a35*a69);
  a70=sin(a69);
  a70=(a34*a70);
  a71=(a29*a70);
  a8=(a8+a71);
  a24=(a24+a31);
  a24=(a18*a24);
  a31=atan2(a66,a39);
  a31=(a36*a31);
  a72=atan(a31);
  a72=(a31-a72);
  a72=(a42*a72);
  a72=(a31-a72);
  a73=atan(a72);
  a73=(a35*a73);
  a74=sin(a73);
  a74=(a34*a74);
  a75=(a24*a74);
  a8=(a8+a75);
  a30=(a30-a8);
  if (res[0]!=0) res[0][4]=a30;
  a30=arg[5]? arg[5][1] : 0;
  a8=arg[1]? arg[1][5] : 0;
  a8=(a30*a8);
  a76=(a11*a16);
  a77=(a46*a53);
  a78=(a76-a77);
  a78=(a78*a32);
  a79=2.;
  a78=(a78/a79);
  a11=(a11*a53);
  a46=(a46*a16);
  a80=(a11+a46);
  a80=(a80*a20);
  a78=(a78+a80);
  a80=(a13*a16);
  a81=(a51*a53);
  a82=(a80-a81);
  a82=(a82*a32);
  a82=(a82/a79);
  a78=(a78-a82);
  a13=(a13*a53);
  a51=(a51*a16);
  a82=(a13+a51);
  a82=(a82*a20);
  a78=(a78+a82);
  a57=(a57*a32);
  a57=(a57/a79);
  a78=(a78+a57);
  a71=(a71*a22);
  a78=(a78-a71);
  a59=(a59*a32);
  a59=(a59/a79);
  a78=(a78-a59);
  a75=(a75*a22);
  a78=(a78-a75);
  a8=(a8-a78);
  if (res[0]!=0) res[0][5]=a8;
  a8=arg[1]? arg[1][6] : 0;
  a78=arg[2]? arg[2][0] : 0;
  a78=(a78-a15);
  a15=arg[5]? arg[5][9] : 0;
  a78=(a78/a15);
  a8=(a8-a78);
  if (res[0]!=0) res[0][6]=a8;
  a8=arg[1]? arg[1][7] : 0;
  a78=arg[2]? arg[2][1] : 0;
  a78=(a78-a12);
  a12=arg[5]? arg[5][8] : 0;
  a78=(a78/a12);
  a8=(a8-a78);
  if (res[0]!=0) res[0][7]=a8;
  a8=arg[1]? arg[1][8] : 0;
  a78=arg[2]? arg[2][2] : 0;
  a78=(a78-a10);
  a78=(a78/a12);
  a8=(a8-a78);
  if (res[0]!=0) res[0][8]=a8;
  a8=arg[1]? arg[1][9] : 0;
  a78=arg[2]? arg[2][3] : 0;
  a78=(a78-a58);
  a78=(a78/a12);
  a8=(a8-a78);
  if (res[0]!=0) res[0][9]=a8;
  a8=arg[1]? arg[1][10] : 0;
  a78=arg[2]? arg[2][4] : 0;
  a78=(a78-a56);
  a78=(a78/a12);
  a8=(a8-a78);
  if (res[0]!=0) res[0][10]=a8;
  if (res[1]!=0) res[1][0]=a6;
  a4=(-a4);
  if (res[1]!=0) res[1][1]=a4;
  a3=(-a3);
  if (res[1]!=0) res[1][2]=a3;
  a4=(-a2);
  if (res[1]!=0) res[1][3]=a4;
  a25=(a1*a25);
  a25=(a25+a26);
  a21=(a21*a25);
  a7=(a0*a7);
  a26=(a28*a7);
  a4=(a26/a32);
  a6=(a21-a4);
  a6=(a18*a6);
  a6=(a45*a6);
  a44=cos(a44);
  a8=casadi_sq(a37);
  a78=casadi_sq(a40);
  a56=(a8+a78);
  a58=(a37/a56);
  a10=(a36*a58);
  a75=1.;
  a41=casadi_sq(a41);
  a41=(a75+a41);
  a59=(a10/a41);
  a59=(a59-a10);
  a59=(a42*a59);
  a10=(a10+a59);
  a43=casadi_sq(a43);
  a43=(a75+a43);
  a10=(a10/a43);
  a10=(a35*a10);
  a10=(a44*a10);
  a10=(a34*a10);
  a10=(a33*a10);
  a6=(a6-a10);
  a21=(a21+a4);
  a21=(a18*a21);
  a21=(a50*a21);
  a49=cos(a49);
  a10=casadi_sq(a39);
  a8=(a8+a10);
  a37=(a37/a8);
  a59=(a36*a37);
  a47=casadi_sq(a47);
  a47=(a75+a47);
  a79=(a59/a47);
  a79=(a79-a59);
  a79=(a42*a79);
  a59=(a59+a79);
  a48=casadi_sq(a48);
  a48=(a75+a48);
  a59=(a59/a48);
  a59=(a35*a59);
  a59=(a49*a59);
  a59=(a34*a59);
  a59=(a27*a59);
  a21=(a21-a59);
  a59=(a6+a21);
  a79=(a53*a59);
  a62=(a1*a62);
  a62=(a62+a63);
  a61=(a61+a62);
  a61=(a65*a61);
  a65=casadi_sq(a65);
  a65=(a75-a65);
  a64=(a64*a65);
  a60=(a60*a64);
  a61=(a61+a60);
  a79=(a79+a61);
  if (res[1]!=0) res[1][4]=a79;
  a59=(a16*a59);
  a55=(a55*a25);
  a25=(a55-a4);
  a25=(a18*a25);
  a25=(a70*a25);
  a69=cos(a69);
  a79=casadi_sq(a66);
  a78=(a79+a78);
  a61=(a66/a78);
  a60=(a36*a61);
  a67=casadi_sq(a67);
  a67=(a75+a67);
  a64=(a60/a67);
  a64=(a64-a60);
  a64=(a42*a64);
  a60=(a60+a64);
  a68=casadi_sq(a68);
  a68=(a75+a68);
  a60=(a60/a68);
  a60=(a35*a60);
  a60=(a69*a60);
  a60=(a34*a60);
  a60=(a29*a60);
  a25=(a25-a60);
  a59=(a59+a25);
  a55=(a55+a4);
  a55=(a18*a55);
  a55=(a74*a55);
  a73=cos(a73);
  a79=(a79+a10);
  a66=(a66/a79);
  a10=(a36*a66);
  a31=casadi_sq(a31);
  a31=(a75+a31);
  a4=(a10/a31);
  a4=(a4-a10);
  a4=(a42*a4);
  a10=(a10+a4);
  a72=casadi_sq(a72);
  a72=(a75+a72);
  a10=(a10/a72);
  a10=(a35*a10);
  a10=(a73*a10);
  a10=(a34*a10);
  a10=(a24*a10);
  a55=(a55-a10);
  a59=(a59+a55);
  a59=(a7-a59);
  if (res[1]!=0) res[1][5]=a59;
  a59=(a16*a6);
  a59=(a20*a59);
  a6=(a53*a6);
  a6=(a32*a6);
  a6=(a19*a6);
  a59=(a59-a6);
  a6=(a53*a21);
  a6=(a32*a6);
  a6=(a19*a6);
  a59=(a59+a6);
  a21=(a16*a21);
  a21=(a20*a21);
  a59=(a59+a21);
  a25=(a22*a25);
  a59=(a59-a25);
  a55=(a22*a55);
  a59=(a59-a55);
  a59=(-a59);
  if (res[1]!=0) res[1][6]=a59;
  if (res[1]!=0) res[1][7]=a2;
  if (res[1]!=0) res[1][8]=a3;
  a26=(a26/a23);
  a26=(a18*a26);
  a3=(a45*a26);
  a56=(a40/a56);
  a2=(a36*a56);
  a59=(a2/a41);
  a59=(a2-a59);
  a59=(a42*a59);
  a2=(a2-a59);
  a2=(a2/a43);
  a2=(a35*a2);
  a2=(a44*a2);
  a2=(a34*a2);
  a2=(a33*a2);
  a3=(a3+a2);
  a2=(a50*a26);
  a8=(a39/a8);
  a59=(a36*a8);
  a55=(a59/a47);
  a55=(a59-a55);
  a55=(a42*a55);
  a59=(a59-a55);
  a59=(a59/a48);
  a59=(a35*a59);
  a59=(a49*a59);
  a59=(a34*a59);
  a59=(a27*a59);
  a2=(a2+a59);
  a59=(a3+a2);
  a55=(a53*a59);
  a55=(a55-a7);
  if (res[1]!=0) res[1][9]=a55;
  a59=(a16*a59);
  a40=(a40/a78);
  a78=(a36*a40);
  a55=(a78/a67);
  a55=(a78-a55);
  a55=(a42*a55);
  a78=(a78-a55);
  a78=(a78/a68);
  a78=(a35*a78);
  a78=(a69*a78);
  a78=(a34*a78);
  a78=(a29*a78);
  a55=(a70*a26);
  a78=(a78-a55);
  a59=(a59+a78);
  a39=(a39/a79);
  a79=(a36*a39);
  a55=(a79/a31);
  a55=(a79-a55);
  a55=(a42*a55);
  a79=(a79-a55);
  a79=(a79/a72);
  a79=(a35*a79);
  a79=(a73*a79);
  a79=(a34*a79);
  a79=(a24*a79);
  a26=(a74*a26);
  a79=(a79-a26);
  a59=(a59+a79);
  a59=(-a59);
  if (res[1]!=0) res[1][10]=a59;
  a59=(a16*a3);
  a59=(a20*a59);
  a3=(a53*a3);
  a3=(a32*a3);
  a3=(a19*a3);
  a59=(a59-a3);
  a3=(a53*a2);
  a3=(a32*a3);
  a3=(a19*a3);
  a59=(a59+a3);
  a2=(a16*a2);
  a2=(a20*a2);
  a59=(a59+a2);
  a78=(a22*a78);
  a59=(a59-a78);
  a79=(a22*a79);
  a59=(a59-a79);
  a59=(-a59);
  if (res[1]!=0) res[1][11]=a59;
  a59=-1.;
  if (res[1]!=0) res[1][12]=a59;
  a5=(a0*a5);
  a59=(a28*a5);
  a59=(a59/a23);
  a1=(a0*a1);
  a79=(a28*a1);
  a79=(a79/a32);
  a78=(a59-a79);
  a78=(a18*a78);
  a78=(a45*a78);
  a56=(a56*a20);
  a58=(a58*a38);
  a56=(a56-a58);
  a56=(a36*a56);
  a58=(a56/a41);
  a58=(a56-a58);
  a58=(a42*a58);
  a56=(a56-a58);
  a56=(a56/a43);
  a56=(a35*a56);
  a56=(a44*a56);
  a56=(a34*a56);
  a56=(a33*a56);
  a78=(a78+a56);
  a56=(a59+a79);
  a56=(a18*a56);
  a58=(a50*a56);
  a8=(a8*a20);
  a37=(a37*a38);
  a8=(a8+a37);
  a8=(a36*a8);
  a37=(a8/a47);
  a37=(a8-a37);
  a37=(a42*a37);
  a8=(a8-a37);
  a8=(a8/a48);
  a8=(a35*a8);
  a8=(a49*a8);
  a8=(a34*a8);
  a8=(a27*a8);
  a58=(a58+a8);
  a8=(a78+a58);
  a37=(a53*a8);
  a37=(a37-a5);
  if (res[1]!=0) res[1][13]=a37;
  a8=(a16*a8);
  a56=(a70*a56);
  a40=(a40*a22);
  a61=(a61*a38);
  a40=(a40+a61);
  a40=(a36*a40);
  a67=(a40/a67);
  a67=(a67-a40);
  a67=(a42*a67);
  a40=(a40+a67);
  a40=(a40/a68);
  a40=(a35*a40);
  a69=(a69*a40);
  a69=(a34*a69);
  a29=(a29*a69);
  a56=(a56+a29);
  a8=(a8-a56);
  a79=(a79-a59);
  a79=(a18*a79);
  a79=(a74*a79);
  a66=(a66*a38);
  a39=(a39*a22);
  a66=(a66-a39);
  a66=(a36*a66);
  a31=(a66/a31);
  a31=(a66-a31);
  a31=(a42*a31);
  a66=(a66-a31);
  a66=(a66/a72);
  a66=(a35*a66);
  a73=(a73*a66);
  a73=(a34*a73);
  a24=(a24*a73);
  a79=(a79+a24);
  a8=(a8+a79);
  a1=(a1-a8);
  if (res[1]!=0) res[1][14]=a1;
  a1=(a16*a78);
  a1=(a20*a1);
  a78=(a53*a78);
  a78=(a32*a78);
  a78=(a19*a78);
  a1=(a1-a78);
  a78=(a53*a58);
  a78=(a32*a78);
  a78=(a19*a78);
  a1=(a1+a78);
  a58=(a16*a58);
  a58=(a20*a58);
  a1=(a1+a58);
  a56=(a22*a56);
  a1=(a1+a56);
  a79=(a22*a79);
  a1=(a1-a79);
  a1=(-a1);
  if (res[1]!=0) res[1][15]=a1;
  a41=(a36/a41);
  a41=(a41-a36);
  a41=(a42*a41);
  a41=(a36+a41);
  a41=(a41/a43);
  a41=(a35*a41);
  a44=(a44*a41);
  a44=(a34*a44);
  a33=(a33*a44);
  a47=(a36/a47);
  a47=(a47-a36);
  a42=(a42*a47);
  a36=(a36+a42);
  a36=(a36/a48);
  a35=(a35*a36);
  a49=(a49*a35);
  a34=(a34*a49);
  a27=(a27*a34);
  a34=(a33+a27);
  a49=(a53*a34);
  a52=(a52-a49);
  a14=(a14+a52);
  if (res[1]!=0) res[1][16]=a14;
  a34=(a16*a34);
  a34=(a34+a54);
  a17=(a17-a34);
  a17=(-a17);
  if (res[1]!=0) res[1][17]=a17;
  a17=(a16*a33);
  a17=(a17+a77);
  a76=(a76-a17);
  a76=(a20*a76);
  a33=(a53*a33);
  a46=(a46-a33);
  a11=(a11+a46);
  a11=(a32*a11);
  a11=(a19*a11);
  a76=(a76-a11);
  a11=(a53*a27);
  a51=(a51-a11);
  a13=(a13+a51);
  a13=(a32*a13);
  a13=(a19*a13);
  a76=(a76+a13);
  a27=(a16*a27);
  a27=(a27+a81);
  a80=(a80-a27);
  a80=(a20*a80);
  a76=(a76+a80);
  a76=(-a76);
  if (res[1]!=0) res[1][18]=a76;
  a15=(1./a15);
  if (res[1]!=0) res[1][19]=a15;
  a76=(a16*a9);
  a80=(-a76);
  if (res[1]!=0) res[1][20]=a80;
  a27=(a53*a9);
  a81=(-a27);
  if (res[1]!=0) res[1][21]=a81;
  a27=(a20*a27);
  a76=(a32*a76);
  a76=(a19*a76);
  a13=(a27-a76);
  a13=(-a13);
  if (res[1]!=0) res[1][22]=a13;
  a12=(1./a12);
  if (res[1]!=0) res[1][23]=a12;
  if (res[1]!=0) res[1][24]=a80;
  if (res[1]!=0) res[1][25]=a81;
  a76=(a76+a27);
  a76=(-a76);
  if (res[1]!=0) res[1][26]=a76;
  if (res[1]!=0) res[1][27]=a12;
  a76=(-a9);
  if (res[1]!=0) res[1][28]=a76;
  a9=(a32*a9);
  a9=(a19*a9);
  if (res[1]!=0) res[1][29]=a9;
  if (res[1]!=0) res[1][30]=a12;
  if (res[1]!=0) res[1][31]=a76;
  a9=(-a9);
  if (res[1]!=0) res[1][32]=a9;
  if (res[1]!=0) res[1][33]=a12;
  if (res[2]!=0) res[2][0]=a75;
  if (res[2]!=0) res[2][1]=a75;
  if (res[2]!=0) res[2][2]=a75;
  a28=(a28*a0);
  a23=(a28/a23);
  a23=(a18*a23);
  a9=(a45*a23);
  a76=(a50*a23);
  a27=(a9+a76);
  a81=(a53*a27);
  a81=(a0-a81);
  if (res[2]!=0) res[2][3]=a81;
  a81=(a70*a23);
  a27=(a16*a27);
  a27=(a81-a27);
  a23=(a74*a23);
  a27=(a27+a23);
  a27=(-a27);
  if (res[2]!=0) res[2][4]=a27;
  a27=(a53*a9);
  a27=(a32*a27);
  a27=(a19*a27);
  a9=(a16*a9);
  a9=(a20*a9);
  a27=(a27-a9);
  a9=(a53*a76);
  a9=(a32*a9);
  a9=(a19*a9);
  a27=(a27-a9);
  a76=(a16*a76);
  a76=(a20*a76);
  a27=(a27-a76);
  a81=(a22*a81);
  a27=(a27-a81);
  a23=(a22*a23);
  a27=(a27-a23);
  a27=(-a27);
  if (res[2]!=0) res[2][5]=a27;
  a28=(a28/a32);
  a18=(a18*a28);
  a50=(a50*a18);
  a45=(a45*a18);
  a28=(a50-a45);
  a27=(a53*a28);
  if (res[2]!=0) res[2][6]=a27;
  a28=(a16*a28);
  a70=(a70*a18);
  a28=(a28-a70);
  a74=(a74*a18);
  a28=(a28+a74);
  a0=(a0-a28);
  if (res[2]!=0) res[2][7]=a0;
  a0=(a53*a45);
  a0=(a32*a0);
  a0=(a19*a0);
  a45=(a16*a45);
  a45=(a20*a45);
  a0=(a0-a45);
  a53=(a53*a50);
  a32=(a32*a53);
  a19=(a19*a32);
  a0=(a0+a19);
  a16=(a16*a50);
  a20=(a20*a16);
  a0=(a0+a20);
  a70=(a22*a70);
  a0=(a0+a70);
  a22=(a22*a74);
  a0=(a0-a22);
  a0=(-a0);
  if (res[2]!=0) res[2][8]=a0;
  if (res[2]!=0) res[2][9]=a30;
  if (res[2]!=0) res[2][10]=a75;
  if (res[2]!=0) res[2][11]=a75;
  if (res[2]!=0) res[2][12]=a75;
  if (res[2]!=0) res[2][13]=a75;
  if (res[2]!=0) res[2][14]=a75;
  a15=(-a15);
  if (res[3]!=0) res[3][0]=a15;
  a12=(-a12);
  if (res[3]!=0) res[3][1]=a12;
  if (res[3]!=0) res[3][2]=a12;
  if (res[3]!=0) res[3][3]=a12;
  if (res[3]!=0) res[3][4]=a12;
  return 0;
}

CASADI_SYMBOL_EXPORT int dyn6_impl_dae_fun_jac_x_xdot_u(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int dyn6_impl_dae_fun_jac_x_xdot_u_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int dyn6_impl_dae_fun_jac_x_xdot_u_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void dyn6_impl_dae_fun_jac_x_xdot_u_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int dyn6_impl_dae_fun_jac_x_xdot_u_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void dyn6_impl_dae_fun_jac_x_xdot_u_release(int mem) {
}

CASADI_SYMBOL_EXPORT void dyn6_impl_dae_fun_jac_x_xdot_u_incref(void) {
}

CASADI_SYMBOL_EXPORT void dyn6_impl_dae_fun_jac_x_xdot_u_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int dyn6_impl_dae_fun_jac_x_xdot_u_n_in(void) { return 6;}

CASADI_SYMBOL_EXPORT casadi_int dyn6_impl_dae_fun_jac_x_xdot_u_n_out(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_real dyn6_impl_dae_fun_jac_x_xdot_u_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* dyn6_impl_dae_fun_jac_x_xdot_u_name_in(casadi_int i) {
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

CASADI_SYMBOL_EXPORT const char* dyn6_impl_dae_fun_jac_x_xdot_u_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    case 3: return "o3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* dyn6_impl_dae_fun_jac_x_xdot_u_sparsity_in(casadi_int i) {
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

CASADI_SYMBOL_EXPORT const casadi_int* dyn6_impl_dae_fun_jac_x_xdot_u_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s4;
    case 2: return casadi_s5;
    case 3: return casadi_s6;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int dyn6_impl_dae_fun_jac_x_xdot_u_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 6;
  if (sz_res) *sz_res = 4;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

CASADI_SYMBOL_EXPORT int dyn6_impl_dae_fun_jac_x_xdot_u_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 6*sizeof(const casadi_real*);
  if (sz_res) *sz_res = 4*sizeof(casadi_real*);
  if (sz_iw) *sz_iw = 0*sizeof(casadi_int);
  if (sz_w) *sz_w = 0*sizeof(casadi_real);
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
