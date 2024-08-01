#pragma once

#include <vector>
#include <spdlog/spdlog.h>
#include <igcclib/vision/igcclib_opencv.hpp>
#include <igcclib/vision/igcclib_image_processing.hpp>

inline cv::Mat do_resize(const cv::Mat& img, int width, int height){    
    spdlog::info("Resizing image from {}x{} to {}x{}", img.cols, img.rows, width, height);
    
    // resize it
    igcclib::ImageRGBA_f _resized_img;
    igcclib::ImageRGBA_f _img;
    _img.from_linear_buffer(std::vector<uint8_t>(img.data, img.data + img.total() * img.elemSize()), 
            img.cols, img.rows, img.channels(), 0, 255, 0, 1);
    igcclib::resize_image(_img, _resized_img, width, height);

    // combine it back to cv::Mat
    std::vector<uint8_t> resized_img_data;
    _resized_img.to_linear_buffer(resized_img_data, 0, 1, 0, 255);
    int cvtype = img.channels() == 3 ? CV_8UC3 : CV_8UC4;
    cv::Mat resized_img = cv::Mat(_resized_img.get_height(), _resized_img.get_width(), cvtype, resized_img_data.data()).clone();

    return resized_img;
}