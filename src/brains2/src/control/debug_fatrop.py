# Copyright 2025 Tudor Oancea, Mateo Berthet
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

import casadi as ca
import pylab as plt


actual = ca.Sparsity.from_file("debug_fatrop_actual.mtx")

A = ca.Sparsity.from_file("debug_fatrop_A.mtx")
B = ca.Sparsity.from_file("debug_fatrop_B.mtx")
C = ca.Sparsity.from_file("debug_fatrop_C.mtx")
D = ca.Sparsity.from_file("debug_fatrop_D.mtx")
I = ca.Sparsity.from_file("debug_fatrop_I.mtx")
errors = ca.Sparsity.from_file("debug_fatrop_errors.mtx").row()

plt.figure()
plt.spy(
    A, marker="o", color="r", markersize=5, label="expected A", markerfacecolor="white"
)
plt.spy(
    B, marker="o", color="b", markersize=5, label="expected B", markerfacecolor="white"
)
plt.spy(
    C, marker="o", color="g", markersize=5, label="expected C", markerfacecolor="white"
)
plt.spy(
    D, marker="o", color="y", markersize=5, label="expected D", markerfacecolor="white"
)
plt.spy(
    I, marker="o", color="k", markersize=5, label="expected I", markerfacecolor="white"
)
plt.spy(actual, marker="o", color="k", markersize=2, label="actual")

plt.hlines(errors, 0, A.shape[1], color="gray", linestyle="-", label="offending rows")

plt.title("Debug view of fatrop interface structure detection")
plt.legend()
plt.tight_layout()
plt.show()
