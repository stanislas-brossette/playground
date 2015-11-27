include_directories(/usr/include/eigen3)
include_directories(/usr/include/python2.7)
include_directories(/home/stanislas/profiles/default-x86_64-linux-ubuntu-13.04/install/include)
link_directories(/home/stanislas/profiles/default-x86_64-linux-ubuntu-13.04/install/lib)

FIND_PACKAGE(Boost REQUIRED COMPONENTS unit_test_framework)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)

if(UNIX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -std=c++0x -pedantic")
endif()

enable_testing()

add_executable(foo foo.cpp)
target_link_libraries(foo PGSolver_d boost_unit_test_framework)
add_test(foo foo)
