option(OMPL_BUILD_DEMOS "Build OMPL demos" ON)
add_feature_info(OMPL_BUILD_DEMOS "${OMPL_BUILD_DEMOS}" "Whether to build the OMPL demo programs.")

if (OMPL_BUILD_DEMOS)
    file(TO_NATIVE_PATH "${CMAKE_SOURCE_DIR}/resources/ppm" TEST_RESOURCES_DIR)
    string(REPLACE "\\" "\\\\" TEST_RESOURCES_DIR "${TEST_RESOURCES_DIR}")
    add_definitions(-DTEST_RESOURCES_DIR=\"${TEST_RESOURCES_DIR}\")

    macro(add_ompl_demo demo_name)
        add_executable(${ARGV})
        target_link_libraries(${demo_name}
            ompl
            ${Boost_FILESYSTEM_LIBRARY}
            ${Boost_SYSTEM_LIBRARY}
            ${Boost_PROGRAM_OPTIONS_LIBRARY})
    endmacro(add_ompl_demo)

    add_ompl_demo(demo_Point2DPlanning Point2DPlanning.cpp)
    add_ompl_demo(demo_tbit tbit.cpp)

endif(OMPL_BUILD_DEMOS)

file(GLOB OMPL_DEMO_CXX_FILES "*.cpp")
install(FILES ${OMPL_DEMO_CXX_FILES}
    DESTINATION "${OMPL_DEMO_INSTALL_DIR}"
    COMPONENT ompl)