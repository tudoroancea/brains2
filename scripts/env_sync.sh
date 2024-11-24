# Copyright (c) 2024, Tudor Oancea, Matteo Berthet
# check that the script is run from the root of the brains2 repository
if [ $(basename "$PWD") != "brains2" ]; then
    echo "Please run this script from the root of the ihm2 repository"
    exit 1
fi
# update the environment
conda-lock install -n brains2 
