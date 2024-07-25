#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/core/operations.hpp>
#include <spdlog/spdlog.h>
#include <igcclib/vision/igcclib_opencv.hpp>
#include <sys/types.h>

int main(){
    cv::Mat img(10,3,CV_8UC3);
    cv::randu(img, 0, 255);
    std::vector<int> x = {1,2,3,4,5,6,7,8,9,10};
    std::cout<<img<<std::endl;
    
    return 0;
}