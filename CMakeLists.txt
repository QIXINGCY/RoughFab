#cmake最低版本
cmake_minimum_required(VERSION 3.16)
#项目名称
project(RoughFab)

# 在CMake中查找vcpkg的安装目录
set(VCPKG_ROOT "/path/to/vcpkg" CACHE PATH "Vcpkg root directory")
#opencv路径
set(OpenCV_DIR "E:/Code/library/opencv/build")
#CMAKE路径定义
set(CMAKE_TOOLCHAIN_FILE "${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake" CACHE STRING "Vcpkg toolchain file")
set(CMAKE_PREFIX_PATH ${VCPKG_ROOT}/installed/x64-windows)  
set(CGAL_DIR E:/library/CGAL/CGAL-5.5.2)
set(GMP_DIR ${VCPKG_ROOT}/installed/x64-windows/share/gmp)
#set(ENV{PKG_CONFIG_PATH} ${VCPKG_ROOT}/installed/x64-windows/share/pkgconfig/bin)

# Add your project files
file(GLOB SRC_FILES *.cpp)

#添加可执行文件
add_executable(${PROJECT_NAME} main.cpp beamsearch.h)
add_executable(test test.cpp)

# 查找并链接OpenCV库
find_package(OpenCV REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS})

# 链接CGAL库
find_package(CGAL REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE CGAL::CGAL)

#链接GMP
find_package(GMP REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE ${GMP_LIBRARIES})

include(ExternalProject)
ExternalProject_Add(
    pgl
    PREFIX ${CMAKE_BINARY_DIR}/third_party/pgl
    GIT_REPOSITORY https://github.com/haisenzhao/personal-geom-lib.git
    CONFIGURE_COMMAND ""
	UPDATE_DISCONNECTED 1
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
    LOG_DOWNLOAD ON
    )
ExternalProject_Get_Property(pgl source_dir)
set(PglIncludeDir ${source_dir})

ExternalProject_Add(
    glm
    PREFIX ${CMAKE_BINARY_DIR}/third_party/glm
    GIT_REPOSITORY https://github.com/g-truc/glm.git
    CONFIGURE_COMMAND ""
	UPDATE_DISCONNECTED 1
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
    LOG_DOWNLOAD ON
    )
ExternalProject_Get_Property(glm source_dir)
set(GlmIncludeDir ${source_dir})

ExternalProject_Add(
    eigen
    PREFIX ${CMAKE_BINARY_DIR}/third_party/eigen
    GIT_REPOSITORY https://github.com/libigl/eigen.git
    CONFIGURE_COMMAND ""
	UPDATE_DISCONNECTED 1
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
    LOG_DOWNLOAD ON
    )
ExternalProject_Get_Property(eigen source_dir)
set(EigenIncludeDir ${source_dir})


include_directories(${PglIncludeDir} ${GlmIncludeDir} ${EigenIncludeDir})

add_dependencies(${PROJECT_NAME} pgl)
add_dependencies(${PROJECT_NAME} glm)
add_dependencies(${PROJECT_NAME} eigen)