/* This file was automatically generated by CasADi 3.6.6.
 *  It consists of: 
 *   1) content generated by CasADi runtime: not copyrighted
 *   2) template code copied from CasADi source: permissively licensed (MIT-0)
 *   3) user code: owned by the user
 *
 */
#ifdef __cplusplus
extern "C" {
#endif

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

#include <casadi/mem.h>
int accels(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
int accels_alloc_mem(void);
int accels_init_mem(int mem);
void accels_free_mem(int mem);
int accels_checkout(void);
void accels_release(int mem);
void accels_incref(void);
void accels_decref(void);
casadi_int accels_n_in(void);
casadi_int accels_n_out(void);
casadi_real accels_default_in(casadi_int i);
const char* accels_name_in(casadi_int i);
const char* accels_name_out(casadi_int i);
const casadi_int* accels_sparsity_in(casadi_int i);
const casadi_int* accels_sparsity_out(casadi_int i);
int accels_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
int accels_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define accels_SZ_ARG 2
#define accels_SZ_RES 1
#define accels_SZ_IW 0
#define accels_SZ_W 7
casadi_functions* accels_functions(void);
#ifdef __cplusplus
} /* extern "C" */
#endif