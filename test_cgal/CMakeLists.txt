message(STATUS "// =======================================")
message(STATUS "// Adding project test_cgal...")
message(STATUS "// =======================================")

project(test_cgal)

find_package(CGAL REQUIRED COMPONENTS Core)

set(CGAL_DEMO_CXX_STD "17" CACHE STRING "C++ standard to use")

message(STATUS "    CGAL_DEMO_CXX_STD is ${CGAL_DEMO_CXX_STD}.")

set(DEMOS
    demo_CGAL_minkowski_sum
)

foreach(PROGRAM ${DEMOS})
    message(STATUS "    Adding executable: ${PROGRAM}")

    add_executable(${PROGRAM} ${PROGRAM}.cpp)

    set_target_properties(${PROGRAM} PROPERTIES
        CXX_STANDARD ${CGAL_DEMO_CXX_STD}
        CXX_STANDARD_REQUIRED ON
    )

    target_link_libraries(${PROGRAM} CGAL::CGAL CGAL::CGAL_Core)
endforeach(PROGRAM)