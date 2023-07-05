#cmake��Ͱ汾
cmake_minimum_required(VERSION 3.16)
#��Ŀ����
project(RoughFab)

#opencv·��
set(OpenCV_DIR "E:/Code/library/opencv/build")
#CMAKE·������
set(CMAKE_TOOLCHAIN_FILE "${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake" CACHE STRING "Vcpkg toolchain file")
set(CMAKE_PREFIX_PATH ${VCPKG_ROOT}/installed/x64-windows)  
set(CGAL_DIR E:/library/CGAL/CGAL-5.5.2)
set(GMP_DIR E:/software/vcpkg/installed/x64-windows/share/gmp)
#set(ENV{PKG_CONFIG_PATH} "E:/software/vcpkg/installed/x64-windows/share/pkgconfig/bin")

# Add your project files
file(GLOB SRC_FILES *.cpp)

#���ӿ�ִ���ļ�
add_executable(${PROJECT_NAME} ${SRC_FILES})

# ���Ҳ�����OpenCV��
find_package(OpenCV REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS})

# ����CGAL��
find_package(CGAL REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE CGAL::CGAL)

#����GMP
find_package(GMP REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE ${GMP_LIBRARIES})