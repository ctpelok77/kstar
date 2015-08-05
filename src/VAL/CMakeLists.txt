cmake_minimum_required(VERSION 2.8.3)

project(validate)

find_package(BISON)
find_package(FLEX)

if(NOT BISON_FOUND OR NOT FLEX_FOUND OR NOT FLEX_INCLUDE_DIR)
    message("FLEX or BISON where not found, so VAL won't be build.")
    return()
endif()

set(BISON_COMPILE_FLAGS "")
if (BISON_VERSION VERSION_EQUAL 2.5 OR BISON_VERSION VERSION_GREATER 2.5)
    set(BISON_COMPILE_FLAGS "-Wno-conflicts-sr -Wno-conflicts-rr")
endif()
set(FLEX_COMPILE_FLAGS "-+")

if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ansi -Wall -Werror")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-but-set-variable")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-maybe-uninitialized")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-local-typedefs")
elseif(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-private-field")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-tautological-compare")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-parentheses-equality")
    execute_process(
        COMMAND ${CMAKE_CXX_COMPILER} --version
        OUTPUT_VARIABLE _CLANG_VERSION_TEXT)
    string(
        REGEX REPLACE ".*clang version ([0-9]+\\.[0-9]+\\.[0-9]+).*" "\\1"
        CLANG_VERSION_STRING ${_CLANG_VERSION_TEXT})
    if(CLANG_VERSION_STRING VERSION_EQUAL 3.5 OR CLANG_VERSION_STRING VERSION_GREATER 3.5)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-absolute-value")
    endif()
elseif(MSVC)
    # enable exceptions
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc")

    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_SCL_SECURE_NO_WARNINGS")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -wd4018")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -wd4800")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -wd4065")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -wd4099")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -wd4101")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -wd4244")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -wd4996")
    set(FLEX_COMPILE_FLAGS "${FLEX_COMPILE_FLAGS} --wincompat")
else()
    message(FATAL_ERROR "Unsupported compiler: ${CMAKE_CXX_COMPILER}")
endif()

include_directories(${CMAKE_CURRENT_BINARY_DIR} ${FLEX_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})

if(BISON_COMPILE_FLAGS)
    bison_target(validate_Parser pddl+.yacc ${CMAKE_CURRENT_BINARY_DIR}/pddl+.cpp
                 COMPILE_FLAGS ${BISON_COMPILE_FLAGS})
else()
    bison_target(validate_Parser pddl+.yacc ${CMAKE_CURRENT_BINARY_DIR}/pddl+.cpp)
endif()
flex_target(validate_Scanner pddl+.lex  ${CMAKE_CURRENT_BINARY_DIR}/lex.yy.cc
            COMPILE_FLAGS ${FLEX_COMPILE_FLAGS})

set_source_files_properties(
    ${CMAKE_CURRENT_BINARY_DIR}/pddl+.cpp
    PROPERTIES OBJECT_DEPENDS
    ${CMAKE_CURRENT_BINARY_DIR}/lex.yy.cc
)

set(VALIDATE_SOURCES
    Action
    DebugWriteController
    Environment
    Events
    Exceptions
    FuncExp
    LaTeXSupport
    macros
    main
    Ownership
    parse_error
    Plan
    Polynomial
    PrettyPrinter
    Proposition
    ptree
    random
    RepairAdvice
    RobustAnalyse
    sStack
    State
    StateObserver
    tDistribution
    TrajectoryConstraints
    typecheck
    Utils
    Validator
    VisitController
    WriteController
    ${CMAKE_CURRENT_BINARY_DIR}/pddl+.cpp
)

add_executable(validate ${VALIDATE_SOURCES})
