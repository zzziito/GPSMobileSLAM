set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "The Open Motion Planning Library (OMPL)")
set(CPACK_PACKAGE_VENDOR "Rice University")
set(CPACK_PACKAGE_CONTACT "Mark Moll <mmoll@rice.edu>")

# component list
set(CPACK_COMPONENTS_ALL ompl)
# display names for components
set(CPACK_COMPONENT_OMPL_DISPLAY_NAME "OMPL library, headers, and demos")

# core library is required
set(CPACK_COMPONENT_OMPL_REQUIRED ON)

set(CPACK_SOURCE_IGNORE_FILES
    ".git"
    "/.vscode"
    "/build/"
    ".pyc$"
    ".pyo$"
    "__pycache__"
    ".so$"
    ".dylib$"
    ".orig$"
    ".log$"
    ".DS_Store"
    "/html/"
    "/bindings/"
    "TODO"
    ".registered$"
    "binding_generator.py$")
set(CPACK_SOURCE_GENERATOR "TGZ;ZIP")
set(CPACK_GENERATOR "TGZ")

if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
    set(CPACK_GENERATOR "DEB;${CPACK_GENERATOR}")
    if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "i686")
        set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "i386")
    endif()
    if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
        set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "amd64")
    endif()
    execute_process(COMMAND "/usr/bin/lsb_release" "-rs"
        OUTPUT_VARIABLE UBUNTU_RELEASE
        OUTPUT_STRIP_TRAILING_WHITESPACE)
    set(CPACK_PACKAGE_FILE_NAME "${PROJECT_NAME}_${PROJECT_VERSION}_${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}-Ubuntu${UBUNTU_RELEASE}")
endif()

include(CPack)
