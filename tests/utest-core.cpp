// test if core module works properly
#include <catch2/catch_test_macros.hpp>
// #include <catch2/matchers/catch_matchers_range_equals.hpp>
// #include <catch2/matchers/catch_matchers_vector.hpp>
#include <catch2/matchers/catch_matchers_all.hpp>
#include <igcclib/core/igcclib_common.hpp>
#include <spdlog/spdlog.h>

using Catch::Matchers::RangeEquals;
using Catch::Matchers::WithinAbs;

auto predicate_float_equal = [](float a, float b) { return std::abs(a - b) < 1e-6; };

auto test_sort_string(std::vector<std::string> strs) {
    igcclib::sort_string_by_natural_order(strs);
    return strs;
}

// create catch2 test cases
TEST_CASE("test something from core", "[core]") {
    REQUIRE(test_sort_string({"1", "2", "3"}) == std::vector<std::string>{"1", "2", "3"});
    REQUIRE(test_sort_string({"1", "10", "2"}) == std::vector<std::string>{"1", "2", "10"});
    REQUIRE_THAT(igcclib::linspace<double>(0, 1, 5), RangeEquals(std::vector<double>{0, 0.25, 0.5, 0.75, 1.0}, predicate_float_equal));
    REQUIRE_THAT(igcclib::linspace<int>(0,10,5), RangeEquals(std::vector<int>{0,2,4,6,8}));
}