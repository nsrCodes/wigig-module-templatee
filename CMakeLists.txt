check_include_file_cxx(stdint.h HAVE_STDINT_H)
if(HAVE_STDINT_H)
    add_definitions(-DHAVE_STDINT_H)
endif()

set(examples_as_tests_sources)
if(${ENABLE_EXAMPLES})
    set(examples_as_tests_sources
        #test/wigig-module-examples-test-suite.cc
        )
endif()

# build_lib(
#     LIBNAME wigig-module
#     SOURCE_FILES model/wigig-module.cc
#                  helper/wigig-module-helper.cc
#     HEADER_FILES model/wigig-module.h
#                  helper/wigig-module-helper.h
#     LIBRARIES_TO_LINK ${libcore}
#     TEST_SOURCES test/wigig-module-test-suite.cc
#                  ${examples_as_tests_sources}
# )


# Gather all source, header, and test files using the GLOB function
file(GLOB HEADER_FILES "*.h" "helper/*.h" "model/*.h")
file(GLOB SOURCE_FILES "*.cc" "helper/*.cc" "model/*.cc")
file(GLOB TEST_SOURCES "test/*.cc")

build_lib(
  LIBNAME wigig-module
  SOURCE_FILES ${SOURCE_FILES}
  HEADER_FILES ${HEADER_FILES}
  LIBRARIES_TO_LINK ${libcore} ${libinternet} ${libmobility} ${libaodv}
  TEST_SOURCES ${TEST_SOURCES}
)

