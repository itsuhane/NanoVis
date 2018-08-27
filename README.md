# NanoVis

## Prequisities

* C++14 compiler
* Eigen
* OpenCV
* git
* CMake (>= 3.0.0)

## Usage

```shell
git submodule add <nanovis-repo> [<submodule-path>]
git submodule update --init --recursive
```

```cmake
cmake_minimum_required(VERSION 3.0.0)

# ...

add_subdirectory(ext/nanovis)

# ...

# add your targets

target_link_libraries(
    <your target>
    PRIVATE
        nanovis
)
```
