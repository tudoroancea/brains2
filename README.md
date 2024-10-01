# brains2

"it's alive"

## setup

The initial setup is easy:

```shell
git clone https://github.com/tudoroancea/brains2.git --recursive
cd brains2
mamba env create -f environment.yml
mamba activate brains2
chmod +x scripts/*.sh
./scripts/build.sh
. install/setup.sh
pip3 install -e $ACADOS_SOURCE_DIR/interfaces/acados_template
```

you can update the mamba environment with

```shell
mamba env update -f environment.yml
```
