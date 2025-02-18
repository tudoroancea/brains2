from icecream import ic
import casadi as ca

opti = ca.Opti()

x = opti.variable()
opti.subject_to(x >= 1)
opti.subject_to(x <= -1)
opti.minimize(x**2)
opti.solver("ipopt")

f = opti.to_function("f", [x], [x])
ic(f)
# ic(f(1.0))
