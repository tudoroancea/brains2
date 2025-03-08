# if we also specified --fix when calling the script, add `--add-missing 'Tudor Oancea, Matteo Berthet' mit`
# store this in a FIX variable

if [ "$1" == "--fix" ]
then FIX="--add-missing 'Tudor Oancea, Matteo Berthet' mit"
else FIX=""
fi
echo $FIX

ament_copyright src/brains2 --exclude \
    src/brains2/src/sim/generated/* \
    src/brains2/src/sim/generated/dyn6_model/* \
    src/brains2/src/sim/generated/kin6_model/* \
    src/brains2/include/brains2/external/* \
    src/brains2/imgui/* \
    src/brains2/imgui/backends/* \
    $FIX
