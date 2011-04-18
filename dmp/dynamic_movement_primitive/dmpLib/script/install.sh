#/bin/sh

make clean
rm -f CMakeCache.txt
# cmake -DLAB_ROOT=`echo $LAB_ROOT` -DMACHTYPE=`echo $MACHTYPE`
cmake .
N_CPUS=$(cat /proc/cpuinfo | grep 'model name' | sed -e 's/.*: //' | wc -l)
make -j$N_CPUS
# make install

