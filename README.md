# Build and Run
### Linux
```
$ cmake --preset local/release && cmake --build build -j $(nproc)
$ ./build/src/gba
```
### MacOS
```
$ cmake --preset local/release && cmake --build build -j $(sysctl -n hw.ncpu)
$ ./build/src/gba
```
