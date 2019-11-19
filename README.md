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

## Download & Compliation


```
# Do not forget the --recursive tag! This project uses submodules a lot
git clone https://github.com/lasagnaphil/animation-project.git --recursive

cd animation-project
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
# For Ubuntu 18.04 LTS or older:
# cmake .. -DCMAKE_CXX_COMPILER=/usr/bin/g++-7 -DCMAKE_BUILD_TYPE=Release
make -j8 animation_project
```

## gengine

For how to use the gengine library, look at the demos.
