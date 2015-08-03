cmake_minimum_required (VERSION 2.6)

project (vsm-mikrokopter)

# Deal with path to SDK
if (NOT DEFINED VSM_SDK_DIR)
    if (DEFINED ENV{VSM_SDK_DIR})
        set(VSM_SDK_DIR $ENV{VSM_SDK_DIR})
    else()
        # Default SDK path
        if(WIN32)
            set(VSM_SDK_DIR "C:/UGCS/vsm-sdk")
        elseif(APPLE)
            set(VSM_SDK_DIR "/Applications/UgCS/vsm-sdk")
        elseif(UNIX)
            set(VSM_SDK_DIR "/opt/ugcs/vsm-sdk")
        endif()
    endif()
endif()

set(UGCS_PACKAGE_SUMMARY "UgCS vehicle specific module (VSM) for Mikrokopter")
set(UGCS_VSM_USER_GUIDE_LATEX_TITLE "Mikrokopter VSM User Guide")

set(CMAKE_VERBOSE_MAKEFILE true)
set(CMAKE_MODULE_PATH "${VSM_SDK_DIR}/share/cmake")
include("ugcs/vsm")

Build_vsm_config(${CMAKE_SOURCE_DIR}/vsm.conf)

include_directories (include)

file(GLOB SOURCES "src/*.cpp" "src/include/*.h")

# Add the executable
add_executable(${CMAKE_PROJECT_NAME} ${SOURCES})

target_link_libraries(${CMAKE_PROJECT_NAME} ${VSM_LIBS})

Add_install_target()

#Add package target.
include(CPack)
