# Pandar40P
## Clone
```
git clone https://github.com/HesaiTechnology/Pandar40P.git --recursive
```
## Build
```
cd <project>
mkdir build
cd build
cmake ..
make
```
## Add to your project
### Cmake
```
add_subdirectory(<path_to>Pandar40P)

include_directories(
	<path_to>Pandar40P/include
	<path_to>Pandar40P/src/Pandar40P/include
)

target_link_libraries(<Your project>
  Pandar40PSDK
)
```
### C++
```
#include "pandar40p_sdk/pandar40p_sdk.h"
```
