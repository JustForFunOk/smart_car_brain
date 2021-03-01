#include "colorizer.h"

Colorizer::Colorizer(/* args */)
{
}

Colorizer::~Colorizer()
{
}

::std::vector<uint8_t> Colorizer::process_frame(const ::std::vector<uint8_t>& _raw_depth_data
                                                uint8_t _pixel_step)
{
    // uint16
    if(_pixel_step == 2)
    {
        auto depth_data = reinterpret_cast<const uint16_t*>(_raw_depth_data.);
        update_histogram();
    }
    
    
}