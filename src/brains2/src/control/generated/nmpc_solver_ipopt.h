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
int nmpc_solver_ipopt(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
int nmpc_solver_ipopt_alloc_mem(void);
int nmpc_solver_ipopt_init_mem(int mem);
void nmpc_solver_ipopt_free_mem(int mem);
int nmpc_solver_ipopt_checkout(void);
void nmpc_solver_ipopt_release(int mem);
void nmpc_solver_ipopt_incref(void);
void nmpc_solver_ipopt_decref(void);
casadi_int nmpc_solver_ipopt_n_in(void);
casadi_int nmpc_solver_ipopt_n_out(void);
casadi_real nmpc_solver_ipopt_default_in(casadi_int i);
const char* nmpc_solver_ipopt_name_in(casadi_int i);
const char* nmpc_solver_ipopt_name_out(casadi_int i);
const casadi_int* nmpc_solver_ipopt_sparsity_in(casadi_int i);
const casadi_int* nmpc_solver_ipopt_sparsity_out(casadi_int i);
int nmpc_solver_ipopt_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
int nmpc_solver_ipopt_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define nmpc_solver_ipopt_SZ_ARG 58
#define nmpc_solver_ipopt_SZ_RES 12
#define nmpc_solver_ipopt_SZ_IW 0
#define nmpc_solver_ipopt_SZ_W 5248
casadi_functions* nmpc_solver_ipopt_functions(void);
#ifdef __cplusplus
} /* extern "C" */
#endif
