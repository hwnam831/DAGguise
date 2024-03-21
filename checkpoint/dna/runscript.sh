#!/bin/bash

export SIM_DESC=0

$GEM5_ROOT/build/X86/gem5.opt \
        --outdir=m5_merge_dna \
	$GEM5_ROOT/configs/example/se.py \
	--cpu-type=AtomicSimpleCPU \
	--num-cpus=1 \
	--mem-type=DRAMSim2 \
	--caches --l1d_size=32kB --l1i_size=32kB \
        --l1d_assoc=8 --l1i_assoc=8 \
	--l2cache --l3cache \
	--l2_size=512kB --l2_assoc=16 \
	--l3_size=1MB --l3_assoc=16 \
        --cpu-clock=2.4GHz --sys-clock=2.4GHz \
	--mem-size=4GB \
        --dramdeviceconfigfile=$GEM5_ROOT/ext/dramsim2/DRAMSim2/ini/DDR3_micron_32M_8B_x8_sg125.ini \
        --dramsystemconfigfile=$GEM5_ROOT/ext/dramsim2/DRAMSim2/system_reg.ini \
	-c "$GEM5_ROOT/sample_programs/dna/src/mrsfast" \
        -o "--search $GEM5_ROOT/sample_programs/dna/dataset/chr3_50K.fa --seq $GEM5_ROOT/sample_programs/dna/readSimulator/chr3_50K_2000.fq" \
	--checkpoint-dir=$GEM5_ROOT/checkpoints/dna/ \
