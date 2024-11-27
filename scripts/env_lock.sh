# Copyright (c) 2024, Tudor Oancea, Matteo Berthet
# check that the script is run from the root of the brains2 repository
if [ $(basename "$PWD") != "brains2" ]; then
    echo "Please run this script from the root of the brains2 repository"
    exit 1
fi
# lock the environment
conda-lock -f env.yml --mamba
