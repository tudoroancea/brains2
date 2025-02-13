# brains2

> _"it's alive"_

## setup

First, install [miniforge3](https://github.com/conda-forge/miniforge), which on Linux and macOS should amount to
```shell
curl -L -O "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh"
bash Miniforge3-$(uname)-$(uname -m).sh
```
You may need to restart your shell to have access to `conda`.

Now to setup the project you can:
```shell
# install conda-lock, which we will use to install the env from a lock file,
# thus improving reproducibility
conda install -n base conda-lock
# clone the project
git clone https://github.com/tudoroancea/brains2.git --recursive
cd brains2
# create the conda environment and activate it
chmod +x scripts/*.sh
conda activate base
./scripts/env_sync.sh
conda deactivate
conda activate brains2
# build ROS (2) workspace and source it
./scripts/build.sh
. install/setup.sh
# install acados' python interface
pip3 install -e $ACADOS_SOURCE_DIR/interfaces/acados_template
```

This should be enough to run the code. To further use the visualization tools using [Foxglove](https://foxglove.dev/),
make sure you have installed `npm` and run the following:
```shell
cd foxglove_extensions
npm install
npm run local-install
```
You can install [Foxglove Desktop](https://foxglove.dev/download), open it and import the provided template json file:
[`foxglove_template.json`](foxglove_template.json).

## howto

### Build the workspace
```shell
./scripts/build.sh
```

### Run unit tests
```shell
./scripts/test.sh
```

### Update the environment
```shell
./scripts/env_sync.sh
```

### Add a new dependency

Prioritize as much as possible dependencies installable via `conda` to ensure
simplicity and reproducibility. You can check if the desired package is
available on [anaconda.org](https://anaconda.org/) and if it is, you can add it
to the `env.yml` file.

When it is not, still try to prioritize reproducible solutions such as git submodules
**pinned at a specific commit** (and not a branch), and simple to install solutions
(that ideally do not require manual installation steps).

> [!NOTE]
> Currently, the only non-conda dependency is `acados`, which was added as a git
submodule in `src` and is built by `colcon`.

You can now lock the environment with:
```shell
conda-lock -f env.yml --mamba
```
and update it with:
```shell
conda-lock install -n brains2
```
You can also accomplish the same with the scripts [`scripts/env_lock.sh`](scripts/env_lock.sh)
and [`scripts/env_sync.sh`](scripts/env_sync.sh), or [`scripts/env_lock_sync.sh`](scripts/env_lock_sync.sh).

> [!NOTE]
> Locking the environment will automatically update all the dependencies to the
> latest compatible versions, because `conda-lock` does not take into account
> the old lock file.
