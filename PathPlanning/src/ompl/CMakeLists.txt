configure_file("${CMAKE_CURRENT_SOURCE_DIR}/config.h.in" "${CMAKE_CURRENT_BINARY_DIR}/config.h")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/config.h"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/ompl"
    COMPONENT ompl)

file(GLOB_RECURSE OMPL_SOURCE_CODE datastructures/*.cpp util/*.cpp base/*.cpp
  geometric/*.cpp control/*.cpp multilevel/*.cpp tools/*.cpp)
file(GLOB_RECURSE OMPL_HEADERS datastructures/*.h util/*.h base/*.h
  geometric/*.h control/*.h multilevel/*.h tools/*.h)

#############################################
# Add enabled extensions to the source code #
#############################################

## replace XXX by the extension
#if (OMPL_EXTENSION_XXX)
#    file(GLOB_RECURSE OMPL_XXX_EXTENSION_SOURCE_CODE extensions/XXX/*.cpp)
#    set(OMPL_SOURCE_CODE ${OMPL_SOURCE_CODE} ${OMPL_XXX_EXTENSION_SOURCE_CODE})
#
##    if additional libraries need to be linked, they are added to manifest.xml for ROS
##    or they are added to the list of OMPL_LINK_LIBRARIES
#
#endif()

if (OMPL_EXTENSION_ODE)
    file(GLOB_RECURSE OMPL_ODE_EXTENSION_SOURCE_CODE extensions/ode/*.cpp)
    file(GLOB_RECURSE OMPL_ODE_EXTENSION_HEADERS extensions/ode/*.h)
    set(OMPL_SOURCE_CODE ${OMPL_SOURCE_CODE} ${OMPL_ODE_EXTENSION_SOURCE_CODE})
    set(OMPL_HEADERS ${OMPL_HEADERS} ${OMPL_ODE_EXTENSION_HEADERS})
    list(APPEND OMPL_LINK_LIBRARIES "${ODE_LIBRARIES}")
endif()

if (OMPL_EXTENSION_TRIANGLE)
    file(GLOB_RECURSE OMPL_TRIANGLE_EXTENSION_SOURCE_CODE extensions/triangle/*.cpp)
    file(GLOB_RECURSE OMPL_TRIANGLE_EXTENSION_HEADERS extensions/triangle/*.h)
    set(OMPL_SOURCE_CODE ${OMPL_SOURCE_CODE} ${OMPL_TRIANGLE_EXTENSION_SOURCE_CODE})
    set(OMPL_HEADERS ${OMPL_HEADERS} ${OMPL_TRIANGLE_EXTENSION_HEADERS})
    list(APPEND OMPL_LINK_LIBRARIES "${TRIANGLE_LIBRARY}")
endif()

source_group("OMPL Source" FILES "${OMPL_SOURCE_CODE}")
source_group("OMPL Headers" FILES "${OMPL_HEADERS}")

# build the library
add_library(ompl SHARED ${OMPL_SOURCE_CODE})

target_link_libraries(ompl
    ${OMPL_LINK_LIBRARIES}
    ${Boost_SERIALIZATION_LIBRARY}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    ${CMAKE_THREAD_LIBS_INIT})

if (OMPL_EXTENSION_ODE)
    if (NOT CMAKE_VERSION VERSION_LESS 3.13)
        target_link_directories(ompl PUBLIC ${ODE_LIBRARY_DIRS})
    else()
        link_directories(ompl ${ODE_LIBRARY_DIRS})
    endif()
endif()

# install the library
install(TARGETS ompl
    DESTINATION ${CMAKE_INSTALL_LIBDIR}
    COMPONENT ompl)