#include "colorizer.h"

namespace smart_car
{

Colorizer::Colorizer()
{
    histogram_ = std::vector<int>(MAX_DEPTH, 0);
    hist_data_ = histogram_.data();
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