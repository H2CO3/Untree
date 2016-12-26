//
// imgproc.cc
// Untree
//
// Created by Arpad Goretity (H2CO3)
// on 16/12/2016
//
// Licensed under the 2-clause BSD License
//

#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cassert>

#include "imgproc.hh"


// Private Methods

png_image GrayscaleImage::make_png_image() {
    png_image img;
    std::memset(&img, 0, sizeof img);
    img.version = PNG_IMAGE_VERSION;
    return img;
}

png_image GrayscaleImage::make_png_write_state() const {
    png_image img = make_png_image();
    img.width = width;
    img.height = height;
    img.format = PNG_FORMAT_LINEAR_Y;
    return img;
}

void GrayscaleImage::complete_read(png_image *img) {
    if (PNG_IMAGE_FAILED(*img)) {
        png_image_free(img);
        return;
    }

    img->format = PNG_FORMAT_LINEAR_Y;

    std::ptrdiff_t stride = PNG_IMAGE_ROW_STRIDE(*img);
    std::ptrdiff_t u16_bufsize = PNG_IMAGE_BUFFER_SIZE(*img, stride);

    width  = img->width;
    height = img->height;

    buf.resize(u16_bufsize / sizeof(std::uint16_t));
    png_image_finish_read(img, nullptr, buf.data(), stride, nullptr);
    png_image_free(img);
}

// Reading images into raw memory buffers

GrayscaleImage::GrayscaleImage(const char *fname) {
    png_image img = GrayscaleImage::make_png_image();
    png_image_begin_read_from_file(&img, fname);
    complete_read(&img);
}

GrayscaleImage::GrayscaleImage(const std::string &fname) : GrayscaleImage(fname.c_str()) {}

GrayscaleImage::GrayscaleImage(std::FILE *file) {
    png_image img = GrayscaleImage::make_png_image();
    png_image_begin_read_from_stdio(&img, file);
    complete_read(&img);
}

GrayscaleImage::GrayscaleImage(std::istream &stream) {
    stream.seekg(0, std::ios_base::end);
    auto size = stream.tellg();
    stream.seekg(0, std::ios_base::beg);

    std::vector<char> bytes(size);
    stream.read(bytes.data(), bytes.size());

    png_image img = GrayscaleImage::make_png_image();
    png_image_begin_read_from_memory(&img, bytes.data(), bytes.size());
    complete_read(&img);
}

GrayscaleImage::GrayscaleImage(const void *data, std::ptrdiff_t size) {
    png_image img = GrayscaleImage::make_png_image();
    png_image_begin_read_from_memory(&img, data, size);
    complete_read(&img);
}

GrayscaleImage::GrayscaleImage(const GrayscaleImage &that) : buf(that.buf),
                                                             width(that.width),
                                                             height(that.height)
{}

GrayscaleImage::GrayscaleImage(GrayscaleImage &&that) : buf(std::move(that.buf)),
                                                        width(that.width),
                                                        height(that.height)
{
    that.width  = 0;
    that.height = 0;
}

GrayscaleImage::GrayscaleImage(std::ptrdiff_t p_width, std::ptrdiff_t p_height) :
    buf(p_width * p_height),
    width(p_width),
    height(p_height)
{}

// Accessing Pixels

std::uint16_t &GrayscaleImage::operator[](Index idx) {
    return buf[idx.y * width + idx.x];
}

std::uint16_t  GrayscaleImage::operator[](Index idx) const {
    return buf[idx.y * width + idx.x];
}

// Writing files from raw memory buffers

bool GrayscaleImage::write(const char *fname) const {
    png_image img = make_png_write_state();
    int status = png_image_write_to_file(&img, fname, true, buf.data(), 0, nullptr);
    png_image_free(&img);
    return status != 0;
}

bool GrayscaleImage::write(const std::string &fname) const {
    return write(fname.c_str());
}

bool GrayscaleImage::write(std::FILE *file) const {
    png_image img = make_png_write_state();
    int status = png_image_write_to_stdio(&img, file, true, buf.data(), 0, nullptr);
    png_image_free(&img);
    return status != 0;
}

bool GrayscaleImage::write(std::ostream &stream) const {
    std::vector<char> bytes;

    if (not write(bytes)) {
        return false;
    }

    stream.write(bytes.data(), bytes.size());

    return stream.good();
}

bool GrayscaleImage::write(std::vector<char> &bytes) const {
    png_image img = make_png_write_state();
    png_alloc_size_t size = 0;

    int status = png_image_write_to_memory(&img, nullptr, &size, true, buf.data(), 0, nullptr);

    if (status != 0) {
        bytes.resize(size);
        status = png_image_write_to_memory(&img, bytes.data(), &size, true, buf.data(), 0, nullptr);
    }

    png_image_free(&img);

    return status != 0;
}

// Applying Filters

void GrayscaleImage::apply_kernel_at(const Kernel &kernel, std::ptrdiff_t x0, std::ptrdiff_t y0) {
    int radius = (kernel.size() - 1) / 2;

    double px = 0;

    for (int i = -radius; i <= radius; i++) {
        for (int j = -radius; j <= radius; j++) {
            std::ptrdiff_t x = x0 + i;
            std::ptrdiff_t y = y0 + j;
            px += kernel[i + radius][j + radius] * (*this)[{x, y}];
        }
    }

    (*this)[{x0, y0}] = px;
}

void GrayscaleImage::apply_kernel(const Kernel &kernel) {
    int radius = (kernel.size() - 1) / 2;

    for (std::ptrdiff_t x = radius; x < width - radius; x++) {
        for (std::ptrdiff_t y = radius; y < height - radius; y++) {
            apply_kernel_at(kernel, x, y);
        }
    }
}

void GrayscaleImage::gaussian_blur(int radius) {
    int diameter = 2 * radius + 1;
    Kernel kernel(diameter, std::vector<double>(diameter));
    double sum = 0;

    for (int i = -radius; i <= radius; i++) {
        for (int j = -radius; j <= radius; j++) {
            double elem = std::exp(-double(i * i + j * j) / radius);
            kernel[i + radius][j + radius] = elem;
            sum += elem;
        }
    }

    for (int i = -radius; i <= radius; i++) {
        for (int j = -radius; j <= radius; j++) {
            kernel[i + radius][j + radius] /= sum;
        }
    }

    apply_kernel(kernel);
}

void GrayscaleImage::box_blur(int radius) {
    int diameter = 2 * radius + 1;
    double element = 1.0 / (diameter * diameter);
    Kernel kernel(diameter, std::vector<double>(diameter, element));
    apply_kernel(kernel);
}
