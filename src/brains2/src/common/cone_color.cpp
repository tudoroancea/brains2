// Copyright 2025 Tudor Oancea, Mateo Berthet
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

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
