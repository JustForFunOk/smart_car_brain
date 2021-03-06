#include "colorizer.h"

namespace smart_car
{

static ColorMap hue{ {
    { 255, 0, 0 },
    { 255, 255, 0 },
    { 0, 255, 0 },
    { 0, 255, 255 },
    { 0, 0, 255 },
    { 255, 0, 255 },
    { 255, 0, 0 },
    } };

static ColorMap jet{ {
    { 0, 0, 255 },
    { 0, 255, 255 },
    { 255, 255, 0 },
    { 255, 0, 0 },
    { 50, 0, 0 },
    } };

static ColorMap classic{ {
    { 30, 77, 203 },
    { 25, 60, 192 },
    { 45, 117, 220 },
    { 204, 108, 191 },
    { 196, 57, 178 },
    { 198, 33, 24 },
    } };

static ColorMap grayscale{ {
    { 255, 255, 255 },
    { 0, 0, 0 },
    } };

static ColorMap inv_grayscale{ {
    { 0, 0, 0 },
    { 255, 255, 255 },
    } };

static ColorMap biomes{ {
    { 0, 0, 204 },
    { 204, 230, 255 },
    { 255, 255, 153 },
    { 170, 255, 128 },
    { 0, 153, 0 },
    { 230, 242, 255 },
    } };

static ColorMap cold{ {
    { 230, 247, 255 },
    { 0, 92, 230 },
    { 0, 179, 179 },
    { 0, 51, 153 },
    { 0, 5, 15 }
    } };

static ColorMap warm{ {
    { 255, 255, 230 },
    { 255, 204, 0 },
    { 255, 136, 77 },
    { 255, 51, 0 },
    { 128, 0, 0 },
    { 10, 0, 0 }
    } };

static ColorMap quantized{ {
    { 255, 255, 255 },
    { 0, 0, 0 },
    }, 6 };

static ColorMap pattern{ {
    { 255, 255, 255 },
    { 0, 0, 0 },
    { 255, 255, 255 },
    { 0, 0, 0 },
    { 255, 255, 255 },
    { 0, 0, 0 },
    { 255, 255, 255 },
    { 0, 0, 0 },
    { 255, 255, 255 },
    { 0, 0, 0 },
    { 255, 255, 255 },
    { 0, 0, 0 },
    { 255, 255, 255 },
    { 0, 0, 0 },
    { 255, 255, 255 },
    { 0, 0, 0 },
    { 255, 255, 255 },
    { 0, 0, 0 },
    { 255, 255, 255 },
    { 0, 0, 0 },
    { 255, 255, 255 },
    { 0, 0, 0 },
    { 255, 255, 255 },
    { 0, 0, 0 },
    { 255, 255, 255 },
    { 0, 0, 0 },
    { 255, 255, 255 },
    { 0, 0, 0 },
    { 255, 255, 255 },
    { 0, 0, 0 },
    { 255, 255, 255 },
    { 0, 0, 0 },
    { 255, 255, 255 },
    { 0, 0, 0 },
    { 255, 255, 255 },
    { 0, 0, 0 },
    { 255, 255, 255 },
    { 0, 0, 0 },
    { 255, 255, 255 },
    { 0, 0, 0 },
    { 255, 255, 255 },
    { 0, 0, 0 },
    { 255, 255, 255 },
    { 0, 0, 0 },
    { 255, 255, 255 },
    { 0, 0, 0 },
    { 255, 255, 255 },
    { 0, 0, 0 },
    { 255, 255, 255 },
    { 0, 0, 0 },
    } };

Colorizer::Colorizer()
{
    histogram_ = std::vector<int>(MAX_DEPTH, 0);
    hist_data_ = histogram_.data();

    color_maps_ = { &jet, &classic, &grayscale, &inv_grayscale, &biomes, &cold, &warm, &quantized, &pattern, &hue };
}

Colorizer::~Colorizer()
{
}

void Colorizer::process_frame(::std::vector<uint8_t>& pixel_data, uint8_t pixel_step)
{
    // uint16
    if(pixel_step == 2)
    {
        auto coloring_function = [&, this](float data) {
            auto hist_data = hist_data_[(int)data];
            auto pixels = (float)hist_data_[MAX_DEPTH - 1];
            return (hist_data / pixels);
        };

        auto pixel_cnt = pixel_data.size() / pixel_step;
        auto raw_depth_data = reinterpret_cast<uint16_t*>(pixel_data.data());
        update_histogram(hist_data_, raw_depth_data, pixel_cnt);

        std::vector<uint8_t> rgb_pixel_data(3*pixel_cnt, 0);
        make_rgb_data<uint16_t>(raw_depth_data, pixel_cnt, rgb_pixel_data.data(), coloring_function);
        pixel_data = rgb_pixel_data;
    }
}

} // namespace smart_car