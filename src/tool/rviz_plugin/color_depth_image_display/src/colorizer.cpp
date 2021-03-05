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

::std::vector<uint8_t> Colorizer::process_frame(const ::std::vector<uint8_t>& _raw_depth_data,
                                                uint8_t _pixel_step)
{
    // uint16
    if(_pixel_step == 2)
    {
        auto depth_data = reinterpret_cast<const uint16_t*>(_raw_depth_data.data());
        update_histogram(hist_data_, depth_data, _raw_depth_data.size() / _pixel_step);
    }
    
    return _raw_depth_data;
}

} // namespace smart_car