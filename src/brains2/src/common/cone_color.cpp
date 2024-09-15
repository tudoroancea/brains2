// Copyright (c) 2024. Tudor Oancea, Matteo Berthet
#include "brains2/common/cone_color.hpp"
#include <string>

bool brains2::common::is_orange(const brains2::common::ConeColor& c) {
    return c == brains2::common::ConeColor::BIG_ORANGE ||
           c == brains2::common::ConeColor::SMALL_ORANGE;
}

brains2::common::ConeColor brains2::common::cone_color_from_string(const std::string& s) {
    if (s == "blue") {
        return brains2::common::ConeColor::BLUE;
    } else if (s == "yellow") {
        return brains2::common::ConeColor::YELLOW;
    } else if (s == "big_orange") {
        return brains2::common::ConeColor::BIG_ORANGE;
    } else if (s == "small_orange") {
        return brains2::common::ConeColor::SMALL_ORANGE;
    } else {
        throw std::runtime_error("invalid cone color " + s);
    }
}

std::string brains2::common::cone_color_to_string(const brains2::common::ConeColor& c) {
    if (c == brains2::common::ConeColor::BLUE) {
        return "blue";
    } else if (c == brains2::common::ConeColor::YELLOW) {
        return "yellow";
    } else if (c == brains2::common::ConeColor::BIG_ORANGE) {
        return "big_orange";
    } else if (c == brains2::common::ConeColor::SMALL_ORANGE) {
        return "small_orange";
    } else {
        throw std::runtime_error("invalid cone color");
    }
}
