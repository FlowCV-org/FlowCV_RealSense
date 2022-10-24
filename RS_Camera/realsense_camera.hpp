//
// RealSense Camera Device Handler
//

#ifndef FLOWCV_REALSENSE_CAMERA_HPP_
#define FLOWCV_REALSENSE_CAMERA_HPP_
#include <iostream>
#include <vector>
#include <string>
#include <librealsense2/rs.hpp>
#include "opencv2/opencv.hpp"
#include "json.hpp"

struct StreamConfig
{
    std::string str_resolution;
    std::string str_stream_name;
    rs2_stream stream_type;
    int width;
    int height;
    std::vector<int> fps_list;
    int fps_idx;
};

struct Property
{
    std::string str_prop;
    rs2_option prop;
    std::vector<std::string> opt_list;
    rs2::option_range range;
    int value;
    bool changed;
    bool is_list;
};

struct FilterOptions
{
    std::string name;
    std::string label;
    std::string description;
    bool is_int;
    bool is_list;
    std::vector<std::string> opt_list;
    float value;
    rs2::option_range range;

    static bool is_all_integers(const rs2::option_range& range);
};

enum StreamAlignment
{
    STREAM_ALIGN_OFF,
    STREAM_ALIGN_DEPTH_TO_COLOR,
    STREAM_ALIGN_COLOR_TO_DEPTH
};

class filter_options
{
  public:
    filter_options(const std::string name, rs2::filter& filter);
    filter_options(filter_options&& other) noexcept ;
    std::string filter_name;
    rs2::filter& filter;
    std::map<rs2_option, FilterOptions> supported_options;
    bool is_enabled;
};

class realsense_camera {
  public:
    realsense_camera();
    void RefreshDeviceList();
    int GetDeviceCount();
    std::string GetDeviceSerial(int index);
    const std::vector<std::string> &GetDeviceList();
    bool InitCamera(int index);
    bool IsInit();
    bool IsReconfiguring() const;
    std::vector<StreamConfig> *GetStreamConfigList(rs2_stream stream_type);
    std::vector<Property> *GetPropertyList(rs2_stream stream_type);
    std::vector<filter_options> *GetFilterList(rs2_stream stream_type);
    void SetProperty(rs2_stream stream_type, rs2_option prop, bool immediate_mode = false);
    void ResetProperties(rs2_stream stream_type);
    int GetProperty(rs2_stream stream_type, rs2_option prop);
    bool EnableStream(StreamConfig& config);
    void DisableStream(rs2_stream stream);
    void ProcessStreams();
    cv::Mat &GetFrame(rs2_stream stream);
    nlohmann::json &GetMetaData();
    void SetStreamAlignment(StreamAlignment align);

  protected:
    void ReconfigureDevice_();
    void ChangeProperties_();

  private:
    std::mutex io_mutex_;
    int active_dev_idx_;
    StreamConfig active_color_cfg_;
    StreamConfig active_depth_cfg_;
    cv::Mat color_frame_;
    cv::Mat depth_frame_;
    std::string rs_dev_serial_;
    std::vector<std::string> camera_name_list_;
    std::vector<rs2::device> camera_dev_list_;
    rs2_intrinsics rsDepthIntrinsics_{};
    rs2_intrinsics rsRGBIntrinsics_{};
    nlohmann::json meta_data_;
    StreamAlignment stream_alignment_;
    bool is_color_enabled_;
    bool is_depth_enabled_;
    bool is_color_streaming_;
    bool is_depth_streaming_;
    bool is_init_;
    bool reconfigure_;
    bool change_props_;
    std::vector<StreamConfig> color_configs_;
    std::vector<Property> color_props_;
    std::vector<StreamConfig> depth_configs_;
    std::vector<Property> depth_props_;
    rs2::config cfg_;
    rs2::pipeline pipe_;
    rs2::pipeline_profile pipe_profile_;
    rs2::device rs_device_;
    rs2::sensor rs_depth_sensor_;
    rs2::sensor rs_color_sensor_;
    std::unique_ptr<rs2::align> align_to_depth;
    std::unique_ptr<rs2::align> align_to_color;
    rs2::decimation_filter rs_dec_filter_;
    rs2::threshold_filter rs_thr_filter_;
    rs2::spatial_filter rs_spat_filter_;
    rs2::temporal_filter rs_temp_filter_;
    rs2::hole_filling_filter rs_hole_fill_filter_;
    std::unique_ptr<rs2::disparity_transform> rs_depth_to_disparity_;
    std::unique_ptr<rs2::disparity_transform> rs_disparity_to_depth_;
    std::vector<filter_options> filters_;
    rs2::frame_queue original_data_;
    rs2::frame_queue filtered_data_;
};

#endif //FLOWCV_REALSENSE_CAMERA_HPP_
