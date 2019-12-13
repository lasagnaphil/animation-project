# Animation Project (SNU 2019)

## Requirements

- Linux or Mac OS environment (Sorry Windows folks)
- A C++17 compiler

If your C++ compiler is old (ex. the default g++ in Ubuntu 18.04 LTS), then you have to install gcc 7.0 separately. For example on Ubuntu:

```
sudo apt-get update
sudo apt-get install -y software-properties-common
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt-get update
sudo apt-get install -y gcc-7 g++-7
```

- SDL2 (Needed for windowing and input)
```
sudo apt-get install -y libsdl2-dev
```

- PhysX

```
git clone https://github.com/lasagnaphil/PhysX
cd PhysX/physx
./generate_projects.sh ( Now plz use option 3 : clang )
cd compiler/linux-debug && make -j8 && cd ../..
cd compiler/linux-release && make -j8 && cd ../../
```

After building the library, you need to pass two variables for CMake: `PHYSX_HOME` and `PXSHARED_HOME`,
which depends on where you've downloaded the library.

In my case, the directory is:

```
PHYSX_HOME = /home/lasagnaphil/dev/PhysX/physx
PXSHARED_HOME = /home/lasagnaphil/dev/PhysX/pxshared
```

## Download & Compliation


```
# Do not forget the --recursive tag! This project uses submodules a lot
git clone https://github.com/lasagnaphil/animation-project.git --recursive

cd animation-project
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DPHYSX_HOME=<dir> -DPXSHARED_HOME=<dir>
# For Ubuntu 18.04 LTS or older:
# cmake .. -DCMAKE_CXX_COMPILER=/usr/bin/g++-7 -DCMAKE_BUILD_TYPE=Release -DPHYSX_HOME=<dir> -DPXSHARED_HOME=<dir>
make -j8 animation_project
```

## gengine

For how to use the gengine library, look at the demos.
