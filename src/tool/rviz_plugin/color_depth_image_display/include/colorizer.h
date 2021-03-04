#pragma once 

#include <vector>
#include <cstdint>  // uint8_t

namespace smart_car
{

class color_map
{
public:
    color_map(std::map<float, float3> map, int steps = 4000) : _map(map)
    {
        initialize(steps);
    }

    color_map(const std::vector<float3>& values, int steps = 4000)
    {
        for (size_t i = 0; i < values.size(); i++)
        {
            _map[(float)i / (values.size() - 1)] = values[i];
        }
        initialize(steps);
    }

    color_map() {}

    inline float3 get(float value) const
    {
        if (_max == _min) return *_data;
        auto t = (value - _min) / (_max - _min);
        t = clamp_val(t, 0.f, 1.f);
        return _data[(int)(t * (_size - 1))];
    }

    float min_key() const { return _min; }
    float max_key() const { return _max; }

    const std::vector<float3>& get_cache() const { return _cache; }

private:
    inline float3 lerp(const float3& a, const float3& b, float t) const
    {
        return b * t + a * (1 - t);
    }

    float3 calc(float value) const
    {
        if (_map.size() == 0) return{ value, value, value };
        // if we have exactly this value in the map, just return it
        if (_map.find(value) != _map.end()) return _map.at(value);
        // if we are beyond the limits, return the first/last element
        if (value < _map.begin()->first)   return _map.begin()->second;
        if (value > _map.rbegin()->first)  return _map.rbegin()->second;

        auto lower = _map.lower_bound(value) == _map.begin() ? _map.begin() : --(_map.lower_bound(value));
        auto upper = _map.upper_bound(value);

        auto t = (value - lower->first) / (upper->first - lower->first);
        auto c1 = lower->second;
        auto c2 = upper->second;
        return lerp(c1, c2, t);
    }

    void initialize(int steps)
    {
        if (_map.size() == 0) return;

        _min = _map.begin()->first;
        _max = _map.rbegin()->first;

        _cache.resize(steps + 1);
        for (int i = 0; i <= steps; i++)
        {
            auto t = (float)i / steps;
            auto x = _min + t*(_max - _min);
            _cache[i] = calc(x);
        }

        // Save size and data to avoid STL checks penalties in DEBUG
        _size = _cache.size();
        _data = _cache.data();
    }

    std::map<float, float3> _map;
    std::vector<float3> _cache;
    float _min, _max;
    size_t _size; float3* _data;
};

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
    ::std::vector<uint8_t> process_frame(const ::std::vector<uint8_t>& _raw_depth_data,
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
