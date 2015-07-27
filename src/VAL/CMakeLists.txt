cmake_minimum_required(VERSION 2.6)

project(validate)

if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ansi -Wall -Werror")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-but-set-variable")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-maybe-uninitialized")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-local-typedefs")
    set(BISON_COMPILE_FLAGS "-Wno-conflicts-sr -Wno-conflicts-rr")
    set(FLEX_COMPILE_FLAGS "-+")
elseif(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_SCL_SECURE_NO_WARNINGS")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -wd4018")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -wd4800")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -wd4065")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -wd4099")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -wd4101")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -wd4244")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -wd4996")
    set(BISON_COMPILE_FLAGS "-Wno-conflicts-sr -Wno-conflicts-rr")
    set(FLEX_COMPILE_FLAGS "-+ --wincompat")
endif()

find_package(BISON REQUIRED)
find_package(FLEX REQUIRED)

include_directories(${CMAKE_CURRENT_BINARY_DIR} ${FLEX_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})

bison_target(validate_Parser pddl+.yacc ${CMAKE_CURRENT_BINARY_DIR}/pddl+.cpp
             COMPILE_FLAGS ${BISON_COMPILE_FLAGS})
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
