cmake_minimum_required(VERSION 3.6)
project(vn300)

set(CMAKE_CXX_STANDARD 11)


include_directories(test/theft)
include_directories(codec)

set(SOURCE_FILES
        codec/vn300_common.c
        codec/vn300_decoder.c
        codec/vn300_encoder.c
        )

set(TEST_SOURCE_FILES
        test/test_decoder.c

        test/theft/theft.c
        test/theft/theft_bloom.c
        test/theft/theft_hash.c
        test/theft/theft_mt.c
        )


add_executable(test_vn300  ${SOURCE_FILES} ${TEST_SOURCE_FILES})