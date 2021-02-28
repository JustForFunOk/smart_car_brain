#pragma once 

#include <vector>

namespace smart_car
{

class Colorizer
{
public:
    Colorizer(/* args */);
    ~Colorizer();

    /**
     * Convert raw depth data to colorful RGB depth data
     * 
     * \param[in] _raw_depth_data the depth data to be processed.
     * \param[in] _pixel_step     the Byte of one pixel. Such as, 16bit(2)
     * \return RGB depth data
     */
    ::std::vector<uint8_t> process_frame(const ::std::vector<uint8_t>& _raw_depth_data
                                         uint8_t _pixel_step);

private:
};

} // namespace smart_car
