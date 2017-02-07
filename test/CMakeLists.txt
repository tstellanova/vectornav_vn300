cmake_minimum_required(VERSION 3.6)
project(vn300)

set(CMAKE_CXX_STANDARD 11)


include_directories(theft)
include_directories(..)

set(SOURCE_FILES
        ../vn300_common.c
        ../vn300_decoder.c
        ../vn300_encoder.c
        )

set(TEST_SOURCE_FILES
        test_decoder.c

        theft/theft.c
        theft/theft_bloom.c
        theft/theft_hash.c
        theft/theft_mt.c
        )


add_executable(test_vn300  ${SOURCE_FILES} ${TEST_SOURCE_FILES})