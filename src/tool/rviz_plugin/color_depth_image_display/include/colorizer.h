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

public:
    static const int MAX_DEPTH = 65536;

private:

    template <typename T>
    static void update_histogram(int* hist, const T* depth_data, int data_cnt)
    {
        memset(hist, 0, MAX_DEPTH * sizeof(int));

        for(auto i = 0; i < data_cnt; ++i)
        {
            T depth_val = depth_data[i];
            int index = static_cast<int>(depth_val);
            hist[index] += 1;
        }

        for(auto i = 2; i < MAX_DEPTH; ++i)
        {
            hist[i] += hist[i-1];  // Build a cumulative histogram for the indices in [1,0xFFFF]
        }
    }


    template<typename T, typename F>
    void make_rgb_data(const T* depth_data, int data_cnt, uint8_t* rgb_data, F coloring_func)
    {
        auto color_map = color_maps_[maps_index_];  // choose convert style
        
        for(auto i = 0; i < data_cnt; ++i)
        {
            auto depth = depth_data[i];
            colorize_pixel(depth, rgb_data+3*i, color_map, coloring_func);
        }
    }

    template<typename T, typename F>
    void colorize_pixel(T depth, uint8_t* rgb_pixel, color_map* color_map, F coloring_func)
    {

    }

private:

    std::vector<int> histogram_;
    int* hist_data_;

    std::vector<color_map*> color_maps_;
    int maps_index_ = 0;
};

} // namespace smart_car
