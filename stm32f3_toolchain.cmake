set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR ARM)

set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_ASM_COMPILER ${CMAKE_C_COMPILER})
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy CACHE INTERNAL "objcopy tool")
set(CMAKE_SIZE_UTIL arm-none-eabi-size CACHE INTERNAL "size tool")

