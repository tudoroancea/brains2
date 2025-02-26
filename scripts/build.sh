# Copyright (c) 2024, Tudor Oancea, Matteo Berthet
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
