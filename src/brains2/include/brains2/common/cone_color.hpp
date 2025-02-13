// Copyright (c) 2024. Tudor Oancea, Matteo Berthet
#ifndef CONE_COLOR_HPP
#define CONE_COLOR_HPP
#include <string>

namespace brains2 {
namespace common {

enum class ConeColor : uint8_t {
    BLUE = 0,
    YELLOW = 1,
    ORANGE = 2,
};

ConeColor cone_color_from_string(const std::string &s);

std::string cone_color_to_string(const ConeColor &c);

}  // namespace common
}  // namespace brains2
#endif  // CONE_COLOR_HPP
