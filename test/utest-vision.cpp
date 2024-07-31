#include <filesystem>
#include <opencv2/core/hal/interface.h>
#include <string>
#include <vector>
#include <catch2/catch_all.hpp>
#include <spdlog/spdlog.h>
#include <igcclib/vision/igcclib_opencv.hpp>
#include <igcclib/vision/igcclib_image_processing.hpp>

// required definitions of data directory in IGCCLIB_DATA_DIR, otherwise raise compile error
#ifndef IGCCLIB_DATA_DIR
    static_assert(0, "IGCCLIB_DATA_DIR is not defined");
#endif

namespace fs = std::filesystem;

const std::string output_dir = "tmp/output/vision";
const std::string data_dir = IGCCLIB_DATA_DIR;
const std::string image_filename = "05673d9a-caba-4f8a-99ef-55763b124d31.png";

// joint data_dir and image_filename
const std::string image_path = fs::path(data_dir) / image_filename;

void do_resize(const cv::Mat& img, int width, int height){
    spdlog::info("do_resize image to {}x{}", width, height);
    // resize it
    igcclib::ImageRGBA_u _resized_img;
    igcclib::ImageRGBA_u _img;
    _img.from_linear_buffer(std::vector<uint8_t>(img.data, img.data + img.total() * img.elemSize()), 
            img.cols, img.rows, img.channels(), 0, 255, 0, 255);
    igcclib::resize_image(_img, _resized_img, width, height);

    // combine it back to cv::Mat
    std::vector<uint8_t> resized_img_data;
    _resized_img.to_linear_buffer(resized_img_data, 0, 255, 0, 255);
    cv::Mat resized_img(_resized_img.get_height(), _resized_img.get_width(), CV_8UC1, resized_img_data.data());
    cv::imwrite(fs::path(output_dir) / "resized.png", resized_img);
    cv::imwrite(fs::path(output_dir) / "original.png", img);
}

TEST_CASE("test vision component", "[vision]") {
    spdlog::info("test vision component");
    
    // create output directory, allow exists
    std::filesystem::create_directories(output_dir);

    // load an image
    cv::Mat img;
    igcclib::imread(image_path, img);
    
    // resize it
    REQUIRE_NOTHROW(do_resize(img, 512, 512));
}