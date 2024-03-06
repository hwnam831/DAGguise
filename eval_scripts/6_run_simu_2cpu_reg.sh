if [[ -z "$GEM5_ROOT" ]]; then
    echo "GEM5_ROOT is not set!" 1>&2
    exit 1
fi

if [[ -z "$SPEC_ROOT" ]]; then
    echo "SPEC_ROOT is not set!" 1>&2
    exit 1
fi

cd $GEM5_ROOT/eval_scripts/simu_single/
python3 generate_runscript.py ../simu_common/docDist_2cpu_regular.sh
cd $SPEC_ROOT/run
bash $GEM5_ROOT/eval_scripts/simu_single/runscript.sh
cd -
