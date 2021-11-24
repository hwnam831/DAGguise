import m5
from m5.objects import *

# libquantum
libquantum = Process()
libquantum.executable = "libquantum"
libquantum.cmd = ["libquantum"] + ["1397", "8"]

# h264ref
h264ref = Process()
h264ref.executable = "h264ref"
h264ref.cmd = ["h264ref"] + ["-d", "foreman_ref_encoder_baseline.cfg"]

# bzip2
bzip2 = Process()
bzip2.executable = "bzip2"
bzip2.cmd = ["bzip2"] + ["input.source", "280"]

# povray
povray = Process()
povray.executable = "povray"
povray.cmd = ["povray"] + ["SPEC-benchmark-ref.ini"]

# GemsFDTD
GemsFDTD = Process()
GemsFDTD.executable = "GemsFDTD"
GemsFDTD.cmd = ["GemsFDTD"] + []

# mcf
mcf = Process()
mcf.executable = "mcf"
mcf.cmd = ["mcf"] + ["inp.in"]

# milc
milc = Process()
milc.executable = "milc"
milc.cmd = ["milc"] + []
milc.input = "su3imp.in"

# tonto
tonto = Process()
tonto.executable = "tonto"
tonto.cmd = ["tonto"] + []

# namd
namd = Process()
namd.executable = "namd"
namd.cmd = ["namd"] + ["--input", "namd.input", "--iterations", "38", "--output", "namd.out"]

# gamess
gamess = Process()
gamess.executable = "gamess"
gamess.cmd = ["gamess"] + []
gamess.input = "cytosine.2.config"

# lbm
lbm = Process()
lbm.executable = "lbm"
lbm.cmd = ["lbm"] + ["3000", "reference.dat", "0", "0", "100_100_130_ldc.of"]

# wrf
wrf = Process()
wrf.executable = "wrf"
wrf.cmd = ["wrf"] + []

# cactusADM
cactusADM = Process()
cactusADM.executable = "cactusADM"
cactusADM.cmd = ["cactusADM"] + ["benchADM.par"]

# specrand_i
specrand_i = Process()
specrand_i.executable = "specrand"
specrand_i.cmd = ["specrand"] + ["1255432124", "234923"]

# hmmer
hmmer = Process()
hmmer.executable = "hmmer"
hmmer.cmd = ["hmmer"] + ["nph3.hmm", "swiss41"]

# soplex
soplex = Process()
soplex.executable = "soplex"
soplex.cmd = ["soplex"] + ["-s1", "-e", "-m45000", "pds-50.mps"]

# zeusmp
zeusmp = Process()
zeusmp.executable = "zeusmp"
zeusmp.cmd = ["zeusmp"] + []

# gcc
gcc = Process()
gcc.executable = "gcc"
gcc.cmd = ["gcc"] + ["166.i", "-o", "166.s"]

# sjeng
sjeng = Process()
sjeng.executable = "sjeng"
sjeng.cmd = ["sjeng"] + ["ref.txt"]

# gromacs
gromacs = Process()
gromacs.executable = "gromacs"
gromacs.cmd = ["gromacs"] + ["-silent", "-deffnm", "gromacs", "-nice", "0", "2"]

# omnetpp
omnetpp = Process()
omnetpp.executable = "omnetpp"
omnetpp.cmd = ["omnetpp"] + ["omnetpp.ini"]

# calculix
calculix = Process()
calculix.executable = "calculix"
calculix.cmd = ["calculix"] + ["-i", "hyperviscoplastic"]

# specrand_f
specrand_f = Process()
specrand_f.executable = "specrand"
specrand_f.cmd = ["specrand"] + ["1255432124", "234923"]

# gobmk
gobmk = Process()
gobmk.executable = "gobmk"
gobmk.cmd = ["gobmk"] + ["--quiet", "--mode", "gtp"]
gobmk.input = "13x13.tst"

# bwaves
bwaves = Process()
bwaves.executable = "bwaves"
bwaves.cmd = ["bwaves"] + ["2"]

# sphinx3
sphinx3 = Process()
sphinx3.executable = "sphinx_livepretend"
sphinx3.cmd = ["sphinx_livepretend"] + ["ctlfile", ".", "args.an4"]

# astar
astar = Process()
astar.executable = "astar"
astar.cmd = ["astar"] + ["BigLakes2048.cfg"]

# leslie3d
leslie3d = Process()
leslie3d.executable = "leslie3d"
leslie3d.cmd = ["leslie3d"] + []
leslie3d.input = "leslie3d.in"

# x264_r
x264_r = Process()
x264_r.executable = "x264_r"
x264_r.cmd = ["x264_r"] + ["--pass", "1", "--stats", "x264_stats.log", "--bitrate", "1000", "--frames", "1000", "-o", "BuckBunny_New.264", "BuckBunny.yuv", "1280x720"]

# omnetpp_r
omnetpp_r = Process()
omnetpp_r.executable = "omnetpp_r"
omnetpp_r.cmd = ["omnetpp_r"] + ["-c", "General", "-r", "0"]

# parest_r
parest_r = Process()
parest_r.executable = "parest_r"
parest_r.cmd = ["parest_r"] + ["ref.prm"]

# namd_r
namd_r = Process()
namd_r.executable = "namd_r"
namd_r.cmd = ["namd_r"] + ["--input", "apoa1.input", "--output", "apoa1.ref.output", "--iterations", "65"]

# roms_r
roms_r = Process()
roms_r.executable = "roms_r"
roms_r.cmd = ["roms_r"] + []
roms_r.input = "ocean_benchmark2.in.x"

# cam4_r
cam4_r = Process()
cam4_r.executable = "cam4_r"
cam4_r.cmd = ["cam4_r"] + []

# xalancbmk_r
xalancbmk_r = Process()
xalancbmk_r.executable = "xalancbmk_r"
xalancbmk_r.cmd = ["xalancbmk_r"] + ["-v", "t5.xml", "xalanc.xsl"]

# deepsjeng_r
deepsjeng_r = Process()
deepsjeng_r.executable = "deepsjeng_r"
deepsjeng_r.cmd = ["deepsjeng_r"] + ["ref.txt"]

# perlbench_r
perlbench_r = Process()
perlbench_r.executable = "perlbench_r"
perlbench_r.cmd = ["perlbench_r"] + ["-I./lib", "checkspam.pl", "2500", "5", "25", "11", "150", "1", "1", "1", "1"]

# fotonik3d_r
fotonik3d_r = Process()
fotonik3d_r.executable = "fotonik3d_r"
fotonik3d_r.cmd = ["fotonik3d_r"] + []

# wrf_r
wrf_r = Process()
wrf_r.executable = "wrf_r"
wrf_r.cmd = ["wrf_r"] + []

# cactuBSSN_r
cactuBSSN_r = Process()
cactuBSSN_r.executable = "cactuBSSN_r"
cactuBSSN_r.cmd = ["cactuBSSN_r"] + ["spec_ref.par"]

# bwaves_r
bwaves_r = Process()
bwaves_r.executable = "bwaves_r"
bwaves_r.cmd = ["bwaves_r"] + ["bwaves_1"]
bwaves_r.input = "bwaves_1.in"

# xz_r
xz_r = Process()
xz_r.executable = "xz_r"
xz_r.cmd = ["xz_r"] + ["cld.tar.xz", "160", "19cf30ae51eddcbefda78dd06014b4b96281456e078ca7c13e1c0c9e6aaea8dff3efb4ad6b0456697718cede6bd5454852652806a657bb56e07d61128434b474", "59796407", "61004416", "6"]

# povray_r
povray_r = Process()
povray_r.executable = "povray_r"
povray_r.cmd = ["povray_r"] + ["SPEC-benchmark-ref.ini"]

# lbm_r
lbm_r = Process()
lbm_r.executable = "lbm_r"
lbm_r.cmd = ["lbm_r"] + ["3000", "reference.dat", "0", "0", "100_100_130_ldc.of"]

# imagick_r
imagick_r = Process()
imagick_r.executable = "imagick_r"
imagick_r.cmd = ["imagick_r"] + ["-limit", "disk", "0", "refrate_input.tga", "-edge", "41", "-resample", "181%", "-emboss", "31", "-colorspace", "YUV", "-mean-shift", "19x19+15%", "-resize", "30%", "refrate_output.tga"]

# mcf_r
mcf_r = Process()
mcf_r.executable = "mcf_r"
mcf_r.cmd = ["mcf_r"] + ["inp.in"]

# nab_r
nab_r = Process()
nab_r.executable = "nab_r"
nab_r.cmd = ["nab_r"] + ["1am0", "1122214447", "122"]

# gcc_r
gcc_r = Process()
gcc_r.executable = "gcc_r"
gcc_r.cmd = ["gcc_r"] + ["gcc-pp.c", "-O3", "-finline-limit=0", "-fif-conversion", "-fif-conversion2", "-o", "gcc-pp.opts-O3_-finline-limit_0_-fif-conversion_-fif-conversion2.s"]

# blender_r
blender_r = Process()
blender_r.executable = "blender_r"
blender_r.cmd = ["blender_r"] + ["sh3_no_char.blend", "--render-output", "sh3_no_char_", "--threads", "1", "-b", "-F", "RAWTGA", "-s", "849", "-e", "849", "-a"]

# exchange2_r
exchange2_r = Process()
exchange2_r.executable = "exchange2_r"
exchange2_r.cmd = ["exchange2_r"] + ["6"]

# leela_r
leela_r = Process()
leela_r.executable = "leela_r"
leela_r.cmd = ["leela_r"] + ["ref.sgf"]

# x264_r
x264_r_1 = Process(pid=101)
x264_r_1.executable = "x264_r"
x264_r_1.cmd = ["x264_r"] + ["--pass", "1", "--stats", "x264_stats.log", "--bitrate", "1000", "--frames", "1000", "-o", "BuckBunny_New.264", "BuckBunny.yuv", "1280x720"]

# omnetpp_r
omnetpp_r_1 = Process(pid=101)
omnetpp_r_1.executable = "omnetpp_r"
omnetpp_r_1.cmd = ["omnetpp_r"] + ["-c", "General", "-r", "0"]

# parest_r
parest_r_1 = Process(pid=101)
parest_r_1.executable = "parest_r"
parest_r_1.cmd = ["parest_r"] + ["ref.prm"]

# namd_r
namd_r_1 = Process(pid=101)
namd_r_1.executable = "namd_r"
namd_r_1.cmd = ["namd_r"] + ["--input", "apoa1.input", "--output", "apoa1.ref.output", "--iterations", "65"]

# roms_r
roms_r_1 = Process(pid=101)
roms_r_1.executable = "roms_r"
roms_r_1.cmd = ["roms_r"] + []
roms_r_1.input = "ocean_benchmark2.in.x"

# cam4_r
cam4_r_1 = Process(pid=101)
cam4_r_1.executable = "cam4_r"
cam4_r_1.cmd = ["cam4_r"] + []

# xalancbmk_r
xalancbmk_r_1 = Process(pid=101)
xalancbmk_r_1.executable = "xalancbmk_r"
xalancbmk_r_1.cmd = ["xalancbmk_r"] + ["-v", "t5.xml", "xalanc.xsl"]

# deepsjeng_r
deepsjeng_r_1 = Process(pid=101)
deepsjeng_r_1.executable = "deepsjeng_r"
deepsjeng_r_1.cmd = ["deepsjeng_r"] + ["ref.txt"]

# perlbench_r
perlbench_r_1 = Process(pid=101)
perlbench_r_1.executable = "perlbench_r"
perlbench_r_1.cmd = ["perlbench_r"] + ["-I./lib", "checkspam.pl", "2500", "5", "25", "11", "150", "1", "1", "1", "1"]

# fotonik3d_r
fotonik3d_r_1 = Process(pid=101)
fotonik3d_r_1.executable = "fotonik3d_r"
fotonik3d_r_1.cmd = ["fotonik3d_r"] + []

# wrf_r
wrf_r_1 = Process(pid=101)
wrf_r_1.executable = "wrf_r"
wrf_r_1.cmd = ["wrf_r"] + []

# cactuBSSN_r
cactuBSSN_r_1 = Process(pid=101)
cactuBSSN_r_1.executable = "cactuBSSN_r"
cactuBSSN_r_1.cmd = ["cactuBSSN_r"] + ["spec_ref.par"]

# bwaves_r
bwaves_r_1 = Process(pid=101)
bwaves_r_1.executable = "bwaves_r"
bwaves_r_1.cmd = ["bwaves_r"] + ["bwaves_1"]
bwaves_r_1.input = "bwaves_1.in"

# xz_r
xz_r_1 = Process(pid=101)
xz_r_1.executable = "xz_r"
xz_r_1.cmd = ["xz_r"] + ["cld.tar.xz", "160", "19cf30ae51eddcbefda78dd06014b4b96281456e078ca7c13e1c0c9e6aaea8dff3efb4ad6b0456697718cede6bd5454852652806a657bb56e07d61128434b474", "59796407", "61004416", "6"]

# povray_r
povray_r_1 = Process(pid=101)
povray_r_1.executable = "povray_r"
povray_r_1.cmd = ["povray_r"] + ["SPEC-benchmark-ref.ini"]

# lbm_r
lbm_r_1 = Process(pid=101)
lbm_r_1.executable = "lbm_r"
lbm_r_1.cmd = ["lbm_r"] + ["3000", "reference.dat", "0", "0", "100_100_130_ldc.of"]

# imagick_r
imagick_r_1 = Process(pid=101)
imagick_r_1.executable = "imagick_r"
imagick_r_1.cmd = ["imagick_r"] + ["-limit", "disk", "0", "refrate_input.tga", "-edge", "41", "-resample", "181%", "-emboss", "31", "-colorspace", "YUV", "-mean-shift", "19x19+15%", "-resize", "30%", "refrate_output.tga"]

# mcf_r
mcf_r_1 = Process(pid=101)
mcf_r_1.executable = "mcf_r"
mcf_r_1.cmd = ["mcf_r"] + ["inp.in"]

# nab_r
nab_r_1 = Process(pid=101)
nab_r_1.executable = "nab_r"
nab_r_1.cmd = ["nab_r"] + ["1am0", "1122214447", "122"]

# gcc_r
gcc_r_1 = Process(pid=101)
gcc_r_1.executable = "gcc_r"
gcc_r_1.cmd = ["gcc_r"] + ["gcc-pp.c", "-O3", "-finline-limit=0", "-fif-conversion", "-fif-conversion2", "-o", "gcc-pp.opts-O3_-finline-limit_0_-fif-conversion_-fif-conversion2.s"]

# blender_r
blender_r_1 = Process(pid=101)
blender_r_1.executable = "blender_r"
blender_r_1.cmd = ["blender_r"] + ["sh3_no_char.blend", "--render-output", "sh3_no_char_", "--threads", "1", "-b", "-F", "RAWTGA", "-s", "849", "-e", "849", "-a"]

# exchange2_r
exchange2_r_1 = Process(pid=101)
exchange2_r_1.executable = "exchange2_r"
exchange2_r_1.cmd = ["exchange2_r"] + ["6"]

# leela_r
leela_r_1 = Process(pid=101)
leela_r_1.executable = "leela_r"
leela_r_1.cmd = ["leela_r"] + ["ref.sgf"]

# x264_r
x264_r_2 = Process(pid=102)
x264_r_2.executable = "x264_r"
x264_r_2.cmd = ["x264_r"] + ["--pass", "1", "--stats", "x264_stats.log", "--bitrate", "1000", "--frames", "1000", "-o", "BuckBunny_New.264", "BuckBunny.yuv", "1280x720"]

# omnetpp_r
omnetpp_r_2 = Process(pid=102)
omnetpp_r_2.executable = "omnetpp_r"
omnetpp_r_2.cmd = ["omnetpp_r"] + ["-c", "General", "-r", "0"]

# parest_r
parest_r_2 = Process(pid=102)
parest_r_2.executable = "parest_r"
parest_r_2.cmd = ["parest_r"] + ["ref.prm"]

# namd_r
namd_r_2 = Process(pid=102)
namd_r_2.executable = "namd_r"
namd_r_2.cmd = ["namd_r"] + ["--input", "apoa1.input", "--output", "apoa1.ref.output", "--iterations", "65"]

# roms_r
roms_r_2 = Process(pid=102)
roms_r_2.executable = "roms_r"
roms_r_2.cmd = ["roms_r"] + []
roms_r_2.input = "ocean_benchmark2.in.x"

# cam4_r
cam4_r_2 = Process(pid=102)
cam4_r_2.executable = "cam4_r"
cam4_r_2.cmd = ["cam4_r"] + []

# xalancbmk_r
xalancbmk_r_2 = Process(pid=102)
xalancbmk_r_2.executable = "xalancbmk_r"
xalancbmk_r_2.cmd = ["xalancbmk_r"] + ["-v", "t5.xml", "xalanc.xsl"]

# deepsjeng_r
deepsjeng_r_2 = Process(pid=102)
deepsjeng_r_2.executable = "deepsjeng_r"
deepsjeng_r_2.cmd = ["deepsjeng_r"] + ["ref.txt"]

# perlbench_r
perlbench_r_2 = Process(pid=102)
perlbench_r_2.executable = "perlbench_r"
perlbench_r_2.cmd = ["perlbench_r"] + ["-I./lib", "checkspam.pl", "2500", "5", "25", "11", "150", "1", "1", "1", "1"]

# fotonik3d_r
fotonik3d_r_2 = Process(pid=102)
fotonik3d_r_2.executable = "fotonik3d_r"
fotonik3d_r_2.cmd = ["fotonik3d_r"] + []

# wrf_r
wrf_r_2 = Process(pid=102)
wrf_r_2.executable = "wrf_r"
wrf_r_2.cmd = ["wrf_r"] + []

# cactuBSSN_r
cactuBSSN_r_2 = Process(pid=102)
cactuBSSN_r_2.executable = "cactuBSSN_r"
cactuBSSN_r_2.cmd = ["cactuBSSN_r"] + ["spec_ref.par"]

# bwaves_r
bwaves_r_2 = Process(pid=102)
bwaves_r_2.executable = "bwaves_r"
bwaves_r_2.cmd = ["bwaves_r"] + ["bwaves_1"]
bwaves_r_2.input = "bwaves_1.in"

# xz_r
xz_r_2 = Process(pid=102)
xz_r_2.executable = "xz_r"
xz_r_2.cmd = ["xz_r"] + ["cld.tar.xz", "160", "19cf30ae51eddcbefda78dd06014b4b96281456e078ca7c13e1c0c9e6aaea8dff3efb4ad6b0456697718cede6bd5454852652806a657bb56e07d61128434b474", "59796407", "61004416", "6"]

# povray_r
povray_r_2 = Process(pid=102)
povray_r_2.executable = "povray_r"
povray_r_2.cmd = ["povray_r"] + ["SPEC-benchmark-ref.ini"]

# lbm_r
lbm_r_2 = Process(pid=102)
lbm_r_2.executable = "lbm_r"
lbm_r_2.cmd = ["lbm_r"] + ["3000", "reference.dat", "0", "0", "100_100_130_ldc.of"]

# imagick_r
imagick_r_2 = Process(pid=102)
imagick_r_2.executable = "imagick_r"
imagick_r_2.cmd = ["imagick_r"] + ["-limit", "disk", "0", "refrate_input.tga", "-edge", "41", "-resample", "181%", "-emboss", "31", "-colorspace", "YUV", "-mean-shift", "19x19+15%", "-resize", "30%", "refrate_output.tga"]

# mcf_r
mcf_r_2 = Process(pid=102)
mcf_r_2.executable = "mcf_r"
mcf_r_2.cmd = ["mcf_r"] + ["inp.in"]

# nab_r
nab_r_2 = Process(pid=102)
nab_r_2.executable = "nab_r"
nab_r_2.cmd = ["nab_r"] + ["1am0", "1122214447", "122"]

# gcc_r
gcc_r_2 = Process(pid=102)
gcc_r_2.executable = "gcc_r"
gcc_r_2.cmd = ["gcc_r"] + ["gcc-pp.c", "-O3", "-finline-limit=0", "-fif-conversion", "-fif-conversion2", "-o", "gcc-pp.opts-O3_-finline-limit_0_-fif-conversion_-fif-conversion2.s"]

# blender_r
blender_r_2 = Process(pid=102)
blender_r_2.executable = "blender_r"
blender_r_2.cmd = ["blender_r"] + ["sh3_no_char.blend", "--render-output", "sh3_no_char_", "--threads", "1", "-b", "-F", "RAWTGA", "-s", "849", "-e", "849", "-a"]

# exchange2_r
exchange2_r_2 = Process(pid=102)
exchange2_r_2.executable = "exchange2_r"
exchange2_r_2.cmd = ["exchange2_r"] + ["6"]

# leela_r
leela_r_2 = Process(pid=102)
leela_r_2.executable = "leela_r"
leela_r_2.cmd = ["leela_r"] + ["ref.sgf"]

# x264_r
x264_r_3 = Process(pid=103)
x264_r_3.executable = "x264_r"
x264_r_3.cmd = ["x264_r"] + ["--pass", "1", "--stats", "x264_stats.log", "--bitrate", "1000", "--frames", "1000", "-o", "BuckBunny_New.264", "BuckBunny.yuv", "1280x720"]

# omnetpp_r
omnetpp_r_3 = Process(pid=103)
omnetpp_r_3.executable = "omnetpp_r"
omnetpp_r_3.cmd = ["omnetpp_r"] + ["-c", "General", "-r", "0"]

# parest_r
parest_r_3 = Process(pid=103)
parest_r_3.executable = "parest_r"
parest_r_3.cmd = ["parest_r"] + ["ref.prm"]

# namd_r
namd_r_3 = Process(pid=103)
namd_r_3.executable = "namd_r"
namd_r_3.cmd = ["namd_r"] + ["--input", "apoa1.input", "--output", "apoa1.ref.output", "--iterations", "65"]

# roms_r
roms_r_3 = Process(pid=103)
roms_r_3.executable = "roms_r"
roms_r_3.cmd = ["roms_r"] + []
roms_r_3.input = "ocean_benchmark2.in.x"

# cam4_r
cam4_r_3 = Process(pid=103)
cam4_r_3.executable = "cam4_r"
cam4_r_3.cmd = ["cam4_r"] + []

# xalancbmk_r
xalancbmk_r_3 = Process(pid=103)
xalancbmk_r_3.executable = "xalancbmk_r"
xalancbmk_r_3.cmd = ["xalancbmk_r"] + ["-v", "t5.xml", "xalanc.xsl"]

# deepsjeng_r
deepsjeng_r_3 = Process(pid=103)
deepsjeng_r_3.executable = "deepsjeng_r"
deepsjeng_r_3.cmd = ["deepsjeng_r"] + ["ref.txt"]

# perlbench_r
perlbench_r_3 = Process(pid=103)
perlbench_r_3.executable = "perlbench_r"
perlbench_r_3.cmd = ["perlbench_r"] + ["-I./lib", "checkspam.pl", "2500", "5", "25", "11", "150", "1", "1", "1", "1"]

# fotonik3d_r
fotonik3d_r_3 = Process(pid=103)
fotonik3d_r_3.executable = "fotonik3d_r"
fotonik3d_r_3.cmd = ["fotonik3d_r"] + []

# wrf_r
wrf_r_3 = Process(pid=103)
wrf_r_3.executable = "wrf_r"
wrf_r_3.cmd = ["wrf_r"] + []

# cactuBSSN_r
cactuBSSN_r_3 = Process(pid=103)
cactuBSSN_r_3.executable = "cactuBSSN_r"
cactuBSSN_r_3.cmd = ["cactuBSSN_r"] + ["spec_ref.par"]

# bwaves_r
bwaves_r_3 = Process(pid=103)
bwaves_r_3.executable = "bwaves_r"
bwaves_r_3.cmd = ["bwaves_r"] + ["bwaves_1"]
bwaves_r_3.input = "bwaves_1.in"

# xz_r
xz_r_3 = Process(pid=103)
xz_r_3.executable = "xz_r"
xz_r_3.cmd = ["xz_r"] + ["cld.tar.xz", "160", "19cf30ae51eddcbefda78dd06014b4b96281456e078ca7c13e1c0c9e6aaea8dff3efb4ad6b0456697718cede6bd5454852652806a657bb56e07d61128434b474", "59796407", "61004416", "6"]

# povray_r
povray_r_3 = Process(pid=103)
povray_r_3.executable = "povray_r"
povray_r_3.cmd = ["povray_r"] + ["SPEC-benchmark-ref.ini"]

# lbm_r
lbm_r_3 = Process(pid=103)
lbm_r_3.executable = "lbm_r"
lbm_r_3.cmd = ["lbm_r"] + ["3000", "reference.dat", "0", "0", "100_100_130_ldc.of"]

# imagick_r
imagick_r_3 = Process(pid=103)
imagick_r_3.executable = "imagick_r"
imagick_r_3.cmd = ["imagick_r"] + ["-limit", "disk", "0", "refrate_input.tga", "-edge", "41", "-resample", "181%", "-emboss", "31", "-colorspace", "YUV", "-mean-shift", "19x19+15%", "-resize", "30%", "refrate_output.tga"]

# mcf_r
mcf_r_3 = Process(pid=103)
mcf_r_3.executable = "mcf_r"
mcf_r_3.cmd = ["mcf_r"] + ["inp.in"]

# nab_r
nab_r_3 = Process(pid=103)
nab_r_3.executable = "nab_r"
nab_r_3.cmd = ["nab_r"] + ["1am0", "1122214447", "122"]

# gcc_r
gcc_r_3 = Process(pid=103)
gcc_r_3.executable = "gcc_r"
gcc_r_3.cmd = ["gcc_r"] + ["gcc-pp.c", "-O3", "-finline-limit=0", "-fif-conversion", "-fif-conversion2", "-o", "gcc-pp.opts-O3_-finline-limit_0_-fif-conversion_-fif-conversion2.s"]

# blender_r
blender_r_3 = Process(pid=103)
blender_r_3.executable = "blender_r"
blender_r_3.cmd = ["blender_r"] + ["sh3_no_char.blend", "--render-output", "sh3_no_char_", "--threads", "1", "-b", "-F", "RAWTGA", "-s", "849", "-e", "849", "-a"]

# exchange2_r
exchange2_r_3 = Process(pid=103)
exchange2_r_3.executable = "exchange2_r"
exchange2_r_3.cmd = ["exchange2_r"] + ["6"]

# leela_r
leela_r_3 = Process(pid=103)
leela_r_3.executable = "leela_r"
leela_r_3.cmd = ["leela_r"] + ["ref.sgf"]
