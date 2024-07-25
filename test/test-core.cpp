#include <iostream>
#include <spdlog/spdlog.h>
#include <igcclib/core/igcclib_eigen.hpp>

int main(){
    int x = 100;
    spdlog::info("hello world {}", x);
    return 0;
}