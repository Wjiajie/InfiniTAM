###################
# UseBoost.cmake #
###################

set(Boost_FIND_REQUIRED TRUE)
set(Boost_FIND_QUIETLY TRUE)
set(Boost_USE_MULTITHREADED TRUE)
if (CMAKE_CXX_COMPILER_ID EQUAL MSVC)
    set(Boost_USE_STATIC_LIBS TRUE)
    set(Boost_USE_STATIC_RUNTIME OFF)
endif ()
find_package(Boost REQUIRED COMPONENTS filesystem iostreams program_options)
if (CMAKE_CXX_COMPILER_ID MATCHES MSVC)
    target_compile_definitions(${targetname} PUBLIC -DBOOST_ALL_NO_LIB -DBOOST_SYSTEM_NO_DEPRECATED)
endif ()
target_compile_definitions(${targetname} PUBLIC WITH_BOOST)