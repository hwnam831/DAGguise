#!/bin/bash
cd $SPEC_ROOT/run
$GEM5_ROOT/build/X86/gem5.opt \
        --outdir=/home/user/dagguise/eval_scripts/simu_simple/results/bwaves_r \
	$GEM5_ROOT/configs/example/se.py \
	--cpu-type=AtomicSimpleCPU \
	--num-cpus=1 \
	--mem-type=DRAMSim2 \
	--caches --l1d_size=32kB --l1i_size=32kB \
        --l1d_assoc=8 --l1i_assoc=8 \
	--l2cache --l3cache \
	--l2_size=256kB --l2_assoc=16 \
	--l3_size=1MB --l3_assoc=16 \
        --cpu-clock=2.4GHz --sys-clock=2.4GHz \
	--maxtime 1 \
	--mem-size=4GB \
        --dramdeviceconfigfile=$GEM5_ROOT/ext/dramsim2/DRAMSim2/ini/DDR3_micron_32M_8B_x8_sg125.ini \
        --dramsystemconfigfile=$GEM5_ROOT/ext/dramsim2/DRAMSim2/configs/system_reg.ini \
	--benchmark=bwaves_r \
	--dramsim2outputfile=/home/user/dagguise/eval_scripts/simu_simple/results/dram \
	--take-checkpoints=100000000000,100000000000 \
	--checkpoint-dir=$GEM5_ROOT/checkpoint
