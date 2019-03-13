#!/usr/bin/env bash
export LD_LIBRARY_PATH=../ICnet/PSPNet/build/lib:$LD_LIBRARY_PATH
#export LD_LIBRARY_PATH=../ICnet/PSPNet/.build_release/lib:$LD_LIBRARY_PATH
#matlab -nodisplay -r "eval_all;exit" 2>&1 | tee matlab.log
rm -rf build
mkdir build
cd build
cmake ..
make
cd ..
sleep 1

#./lane_assess
#./lane_assess Parameters.xml
