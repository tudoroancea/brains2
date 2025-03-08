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

# build acados
colcon build --packages-select acados --cmake-args -DCMAKE_BUILD_TYPE=Release -Wno-dev -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_C_COMPILER=$CONDA_PREFIX/bin/clang -DCMAKE_CXX_COMPILER=$CONDA_PREFIX/bin/clang++
if [ $? -ne 0 ]; then
    exit $?
fi

# makes sure that the correct python interpreter is called in CMake
PYTHON_EXE=$(which python3)
echo "Using python interpreter: $PYTHON_EXE"

# build brains2
export PYTHONWARNINGS=ignore:::setuptools.command.install,ignore:::setuptools.command.easy_install,ignore:::pkg_resources
colcon build --packages-select brains2 --cmake-args -DPython3_EXECUTABLE=$PYTHON_EXE -DCMAKE_BUILD_TYPE=RelWithDebInfo -Wno-dev -DCMAKE_BUILD_TYPE=Release -Wno-dev -DCMAKE_C_COMPILER=$CONDA_PREFIX/bin/clang -DCMAKE_CXX_COMPILER=$CONDA_PREFIX/bin/clang++
if [ $? -ne 0 ]; then
    exit $?
fi

# setup environment variables (useful for acados' python interface)
echo "export ACADOS_SOURCE_DIR=$(pwd)/src/acados" >> "install/setup.sh"
# echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:install/acados/lib" >> "install/setup.sh"
# echo "export DYLD_LIBRARY_PATH=\$DYLD_LIBRARY_PATH:install/acados/lib" >> "install/setup.sh"
