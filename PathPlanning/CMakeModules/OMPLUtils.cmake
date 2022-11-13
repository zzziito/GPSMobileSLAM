# Computes the link flags and package dependencies for a list of targets. This command:
#
#   target_link_flags(target1 target2 ...)
#
# sets the following variables in the calling context:
#
#   target1_LINK_FLAGS
#   target1_PKG_DEPS
#
# Note that the link flags for *all* targets are combined into two variables.
# The second variable is used for libraries that were found with pkg-config.
# This function is intended for generating pkg-config *.pc files.
function(target_link_flags)
    set(_link_flags "")
    set(_pkg_deps "")
    foreach(_target ${ARGV})
        get_target_property(_link_dirs ${_target} LINK_DIRECTORIES)
        foreach(_dir ${_link_dirs})
            if(NOT ("${_dir}" STREQUAL "_link_dirs-NOTFOUND"))
                list(APPEND _link_flags "-L${_dir}")
            endif()
        endforeach()
        set(_link_dirs "${_link_dirs};${_link_dirs_target}")
        get_target_property(_link_libs ${_target} LINK_LIBRARIES)
        foreach(_lib ${_link_libs})
            get_filename_component(_basename ${_lib} NAME_WE)
            get_filename_component(_ext ${_lib} EXT)
            # add -lfoo to link flags
            if (_lib MATCHES "-l.+")
                list(APPEND _link_flags "${_lib}")
            else()
                # add link flags for dynamic libraries
                if(_ext STREQUAL ${CMAKE_SHARED_LIBRARY_SUFFIX})
                    string(REPLACE ${CMAKE_SHARED_LIBRARY_PREFIX} "" _libname ${_basename})
                    list(APPEND _link_flags "-l${_libname}")
                else()
                    # macOS frameworks, which are also dynamic libraries
                    if (_ext STREQUAL ".framework")
                        list(APPEND _link_flags "-framework ${_basename}")
                    else()
                        # libraries found by pkg-config are just returned as "foo",
                        # not "libfoo.so".
                        if(NOT _ext)
                            list(FIND ARGV ${_basename} _index)
                            if (_index EQUAL -1)
                                # the spot and bddx libraries can be pulled in via a dependency on libspot.pc
                                if(NOT (_basename STREQUAL "bddx" OR _basename STREQUAL "m"))
                                    list(APPEND _pkg_deps "${_basename}")
                                endif()
                          endif()
                        endif()
                    endif()
                endif()
            endif()
        endforeach()
    endforeach()
    list(REMOVE_DUPLICATES _link_flags)
    foreach(_flag ${_link_flags})
        set(_link_flags_str "${_link_flags_str} ${_flag}")
    endforeach()
    string(STRIP "${_link_flags_str}" _link_flags_str)
    set(${ARGV0}_LINK_FLAGS "${_link_flags_str}" PARENT_SCOPE)

    list(REMOVE_DUPLICATES _pkg_deps)
    foreach(_dep ${_pkg_deps})
        set(_pkg_dep_str "${_pkg_dep_str} ${_dep}")
    endforeach()
    string(STRIP "${_pkg_dep_str}" _pkg_dep_str)
    set(${ARGV0}_PKG_DEPS "${_pkg_dep_str}" PARENT_SCOPE)
endfunction()

option(OMPL_VERSIONED_INSTALL "Install header files in include/ompl-X.Y/ompl, where X and Y are the major and minor version numbers" ON)
add_feature_info(OMPL_VERSIONED_INSTALL "${OMPL_VERSIONED_INSTALL}" "Whether to install header files in\n   <prefix>/include/ompl-X.Y/ompl, where X and Y are the major and minor\n   version numbers")
if (OMPL_VERSIONED_INSTALL)
    set(CMAKE_INSTALL_INCLUDEDIR "include/ompl-${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}")
endif()