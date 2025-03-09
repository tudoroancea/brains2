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

# check that the script is run from the root of the brains2 repository
if [ $(basename "$PWD") != "brains2" ]; then
    echo "Please run this script from the root of the brains2 repository"
    exit 1
fi
# check that we use the brains2 conda environment (check CONDA_PREFIX ends with /brains2)
if [ $(basename "$CONDA_PREFIX") != "brains2" ] && [ $(basename "$CONDA_PREFIX") != "brains2-jazzy" ]; then
    echo "Please activate the brains2 conda environment"
    exit 1
fi

# build acados
colcon build --packages-select acados \
             --cmake-args -DCMAKE_BUILD_TYPE=Release \
                          -Wno-dev \
                          -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
                          -DCMAKE_C_COMPILER=$CONDA_PREFIX/bin/clang \
                          -DCMAKE_CXX_COMPILER=$CONDA_PREFIX/bin/clang++
if [ $? -ne 0 ]; then
    exit $?
fi

# build brains2
colcon build --packages-select brains2 \
             --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
                          -DCMAKE_C_COMPILER=$CONDA_PREFIX/bin/clang \
                          -DCMAKE_CXX_COMPILER=$CONDA_PREFIX/bin/clang++ \
                          -Wno-dev  # silence cmake policy warnings
if [ $? -ne 0 ]; then
    exit $?
fi

# setup environment variables for acados' python interface
echo "export ACADOS_SOURCE_DIR=$(pwd)/src/acados" >> "install/setup.sh"
