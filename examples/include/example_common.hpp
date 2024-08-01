#pragma once
#include <string>
#include <stdexcept>
#include <filesystem>

// required definitions of data directory in IGCCLIB_TEST_DATA_DIR, otherwise raise compile error
#ifndef IGCCLIB_TEST_DATA_DIR
    static_assert(0, "IGCCLIB_TEST_DATA_DIR is not defined");
#endif

// required definitions of output directory in IGCCLIB_TEST_OUTPUT_DIR, otherwise raise compile error
#ifndef IGCCLIB_TEST_OUTPUT_DIR
    static_assert(0, "IGCCLIB_TEST_OUTPUT_DIR is not defined");
#endif

class ExampleCommon {
public:
    static std::string get_test_data_dir() {
        return IGCCLIB_TEST_DATA_DIR;
    }

    static std::string get_test_output_dir() {
        return IGCCLIB_TEST_OUTPUT_DIR;
    }

    static std::string get_example_image_path(int index=0){
        switch(index){
            case 0:
                return get_test_data_dir() + "/05673d9a-caba-4f8a-99ef-55763b124d31.png";
            default:
                throw std::invalid_argument("Invalid index");
        }
    }
};