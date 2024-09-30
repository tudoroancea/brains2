# Copyright (c) 2024, Tudor Oancea, Matteo Berthet
# check that the script is run from the root of the ihm2 repository
if [ $(basename "$PWD") != "brains2" ]; then
    echo "Please run this script from the root of the ihm2 repository"
    exit 1
fi

# makes sure that the correct python interpreter is called in CMake
PYTHON_EXE=$(which python3)
echo "Using python interpreter: $PYTHON_EXE"

# run colcon build
PYTHONWARNINGS=ignore:::setuptools.command.install,ignore:::setuptools.command.easy_install,ignore:::pkg_resources colcon build --symlink-install --executor parallel --cmake-args -DPython3_EXECUTABLE=$PYTHON_EXE -DCMAKE_BUILD_TYPE=Debug -Wno-dev
