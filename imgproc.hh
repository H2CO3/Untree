//
// imgproc.hh
// Untree
//
// Created by Arpad Goretity (H2CO3)
// on 16/12/2016
//
// Licensed under the 2-clause BSD License
//

#ifndef UNTREE_IMGPROC_HH
#define UNTREE_IMGPROC_HH

#include <cstddef>
#include <cstdio>
#include <fstream>
#include <vector>
#include <utility>

#include <png.h>


using Kernel = std::vector<std::vector<double>>;

struct Index {
    std::ptrdiff_t x;
    std::ptrdiff_t y;

    bool operator==(Index other) const {
        return x == other.x && y == other.y;
    }

    bool operator!=(Index other) const {
        return not (*this == other);
    }

    std::string to_string() const {
        char buf[256];
        std::snprintf(buf, sizeof buf, "(%4td, %4td)", x, y);
        return buf;
    }
};

namespace std {
template<>
struct hash<Index> {
    std::size_t operator()(Index index) const {
        std::size_t bits = sizeof(index.x) * CHAR_BIT / 2;
        return ((index.x << bits) | (index.x >> bits)) ^ (index.y * std::ptrdiff_t(18446744073709551557ULL));
    }
};
}

struct GrayscaleImage {
	std::vector<std::uint16_t> buf;
	std::ptrdiff_t width;
	std::ptrdiff_t height;

    // Reading Images
    GrayscaleImage(const char *fname);
    GrayscaleImage(const std::string &fname);
    GrayscaleImage(std::FILE *file);
    GrayscaleImage(std::istream &stream);
    GrayscaleImage(const void *data, std::ptrdiff_t size);

    // Other ways of creating images
    GrayscaleImage(const GrayscaleImage &that);
    GrayscaleImage(GrayscaleImage &&that);
    GrayscaleImage(std::ptrdiff_t p_width, std::ptrdiff_t p_height); // all black

    // Accessing Pixels
    std::uint16_t &operator[](Index idx);
    std::uint16_t  operator[](Index idx) const;

    // Writing Images
    bool write(const char *fname) const;
    bool write(const std::string &fname) const;
    bool write(std::FILE *file) const;
    bool write(std::ostream &stream) const;
    bool write(std::vector<char> &bytes) const;

    // Applying Filters
    void apply_kernel_at(const Kernel &kernel, std::ptrdiff_t x0, std::ptrdiff_t y0);
    void apply_kernel(const Kernel &kernel);

    void gaussian_blur(int radius);
    void box_blur(int radius);

private:

    static png_image make_png_image();

    png_image make_png_write_state() const;
    void complete_read(png_image *img);
};

#endif // UNTREE_IMGPROC_HH
