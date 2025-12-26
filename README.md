# Build and Run
### Linux
```
$ cmake --preset release && cmake --build build -j $(nproc)
$ ./build/src/gba
```
### MacOS
```
$ cmake --preset release && cmake --build build -j $(sysctl -n hw.ncpu)
$ ./build/src/gba
```
