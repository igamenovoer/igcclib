#include "example_common.hpp"
#include "example_image_ops.hpp"
#include <filesystem>
#include <igcclib/vision/igcclib_image_processing.hpp>
#include <spdlog/spdlog.h>

namespace  fs = std::filesystem;
int main(){
    auto output_dir = fs::path(ExampleCommon::get_test_output_dir()) / "resize_image";
    fs::create_directories(output_dir);

    auto img_path = ExampleCommon::get_example_image_path();
    cv::Mat img;
    spdlog::info("Loaded image from {}, channels={} rows={} cols={}", img_path, img.channels(), img.rows, img.cols);
    igcclib::imread(img_path, img, igcclib::ImageFormat::RGB);

    auto output = do_resize(img, 512, 512);
    
    igcclib::imwrite(output_dir / "input.jpg", img, igcclib::ImageFormat::RGB);
    spdlog::info("Saved input image to {}", (output_dir / "input.jpg").string());
    
    igcclib::imwrite(output_dir / "output.jpg", output, igcclib::ImageFormat::RGB);
    spdlog::info("Saved resized image to {}", (output_dir / "output.jpg").string());

    return 0;
}