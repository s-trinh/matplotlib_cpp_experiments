cmake_minimum_required(VERSION 3.5)

project(matplotlib_cpp_experiments)
set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(VISP REQUIRED)
find_package(Matplot++ REQUIRED)
find_package(matplotlib_cpp REQUIRED)

set(code_cpp
  src/fplot3_4.cpp  # Matplot++
  src/subplot.cpp   # matplotlib-cpp
  src/lines3d.cpp   # matplotlib-cpp
  src/ibvs-4pts-plotter-continuous-gain-adaptive-plot2d.cpp
  src/ibvs-4pts-plotter-continuous-gain-adaptive-plot3d.cpp
  src/get_matplotlib_buffer_img.cpp
)

foreach(cpp ${code_cpp})
  get_filename_component(target ${cpp} NAME_WE)
  include_directories(${VISP_INCLUDE_DIRS})

  add_executable(${target} ${cpp})

  target_compile_options(${target} PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra>
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
  )

  target_link_libraries(${target} PRIVATE ${OpenCV_LIBS} ${VISP_LIBRARIES} Matplot++::matplot matplotlib_cpp::matplotlib_cpp)
endforeach()
