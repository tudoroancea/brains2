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

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

#include <casadi/mem.h>
int kin6_accels(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
int kin6_accels_alloc_mem(void);
int kin6_accels_init_mem(int mem);
void kin6_accels_free_mem(int mem);
int kin6_accels_checkout(void);
void kin6_accels_release(int mem);
void kin6_accels_incref(void);
void kin6_accels_decref(void);
casadi_int kin6_accels_n_in(void);
casadi_int kin6_accels_n_out(void);
casadi_real kin6_accels_default_in(casadi_int i);
const char* kin6_accels_name_in(casadi_int i);
const char* kin6_accels_name_out(casadi_int i);
const casadi_int* kin6_accels_sparsity_in(casadi_int i);
const casadi_int* kin6_accels_sparsity_out(casadi_int i);
int kin6_accels_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
int kin6_accels_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define kin6_accels_SZ_ARG 2
#define kin6_accels_SZ_RES 1
#define kin6_accels_SZ_IW 0
#define kin6_accels_SZ_W 7
casadi_functions* kin6_accels_functions(void);
#ifdef __cplusplus
} /* extern "C" */
#endif
