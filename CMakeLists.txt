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

build_lib(
    LIBNAME wigig-module
    SOURCE_FILES model/wigig-module.cc
                 helper/wigig-module-helper.cc
    HEADER_FILES model/wigig-module.h
                 helper/wigig-module-helper.h
    LIBRARIES_TO_LINK ${libcore}
    TEST_SOURCES test/wigig-module-test-suite.cc
                 ${examples_as_tests_sources}
)

