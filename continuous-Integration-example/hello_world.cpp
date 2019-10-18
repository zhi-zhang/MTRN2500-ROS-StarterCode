
// SPDX-License-Identifier: Apache-2.0
/**
 *  \brief     Hellow world example to show CI working
 *  \details   We will use fmtlib that will be standardised in c++20
 *  \author    Zhihao Zhang \version   1.0 \date      2019
 *  \copyright Apache-2.0.
 **/

#include "fmt/printf.h"      // https://fmt.dev/6.0.0/api.html
#if defined _MSC_VER
#undef DELETE // work around msvc problem
#endif
#include "rclcpp/rclcpp.hpp" // http://docs.ros2.org/dashing/api/rclcpp/

#include <iostream> // ostream, https://en.cppreference.com/w/cpp/header/iostream

auto main(int argc, char * argv[]) -> int
{
    using namespace std::chrono_literals;
    try
    {
        rclcpp::init(argc, argv);
        fmt::print(std::cout,"Hello world.\n");
        throw std::runtime_error("testing");
    }
    catch (std::exception & e)
    {
        fmt::print(std::cerr, "Error message: {}\n", e.what());
    }
    rclcpp::shutdown();
    return 0;
}