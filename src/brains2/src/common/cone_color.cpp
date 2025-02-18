// Copyright (c) 2024. Tudor Oancea, Matteo Berthet
#include "brains2/common/cone_color.hpp"
#include <stdexcept>
#include <string>

brains2::common::ConeColor brains2::common::cone_color_from_string(const std::string& s) {
    if (s == "blue") {
        return brains2::common::ConeColor::BLUE;
    } else if (s == "yellow") {
        return brains2::common::ConeColor::YELLOW;
    } else if (s == "orange") {
        return brains2::common::ConeColor::ORANGE;
    } else {
        throw std::runtime_error("invalid cone color " + s);
    }
}

std::string brains2::common::cone_color_to_string(const brains2::common::ConeColor& c) {
    if (c == brains2::common::ConeColor::BLUE) {
        return "blue";
    } else if (c == brains2::common::ConeColor::YELLOW) {
        return "yellow";
    } else if (c == brains2::common::ConeColor::ORANGE) {
        return "orange";
    } else {
        throw std::runtime_error("invalid cone color");
    }
}
