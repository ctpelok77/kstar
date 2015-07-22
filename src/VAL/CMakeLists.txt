cmake_minimum_required(VERSION 2.6)

project(validate)

if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ansi -Wall -Werror -Wno-unused-but-set-variable -Wno-maybe-uninitialized -Wno-unused-local-typedefs" )
elseif(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -wd4018 -D_SCL_SECURE_NO_WARNINGS")
endif()

find_package(BISON REQUIRED)
find_package(FLEX REQUIRED)

include_directories(${CMAKE_CURRENT_BINARY_DIR} ${FLEX_INCLUDE_DIRS} .)

bison_target(validate_Parser pddl+.yacc ${CMAKE_CURRENT_BINARY_DIR}/pddl+.cpp)
flex_target(validate_Scanner pddl+.lex  ${CMAKE_CURRENT_BINARY_DIR}/lex.yy.cc COMPILE_FLAGS "-+")

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
    FuncExp
    LaTeXSupport
    main
    Ownership
    Plan
    Polynomial
    PrettyPrinter
    Proposition
    ptree
    random
    RepairAdvice
    RobustAnalyse
    State
    TrajectoryConstraints
    typecheck
    Utils
    Validator
    ${CMAKE_CURRENT_BINARY_DIR}/pddl+.cpp
)

add_executable(validate ${VALIDATE_SOURCES})
