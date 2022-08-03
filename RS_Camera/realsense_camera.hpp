//
// FlowCV Plugin - Realsense Camera
// Written By Richard Wardlow
//

#ifndef FLOWCV_REALSENSE_CAMERA_HPP_
#define FLOWCV_REALSENSE_CAMERA_HPP_
#include <DSPatch.h>
#include "FlowCV_Types.hpp"
#include "json.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <vector>
#include <string>
#include <librealsense2/rs.hpp>

namespace DSPatch::DSPatchables
{
namespace internal
{
class RealsenseCamera;
}

struct VideoMode
{
    int width;
    int height;
};

struct ImageControls
{
    bool auto_exposure;
    int exposure;
    int gain;
};

class DLLEXPORT RealsenseCamera final : public Component
{
  public:
    RealsenseCamera();
    void UpdateGui(void *context, int interface) override;
    bool HasGui(int interface) override;
    std::string GetState() override;
    void SetState(std::string &&json_serialized) override;

  protected:
    void Process_( SignalBus const& inputs, SignalBus& outputs ) override;
    void RefreshDeviceList_();
    bool InitCamera_();
    void SetFilterOptions();
    bool IsValidProfile(rs2_stream stream_type, int width, int height, int fps);

  private:
    std::unique_ptr<internal::RealsenseCamera> p;
    std::mutex io_mutex_;
    cv::Mat frame_, dFrame_;
    std::vector<rs2::device> devices_;
    std::vector<std::string> cam_list_;
    std::vector<std::string> preset_list_;
    rs2::config cfg_;
    rs2::pipeline pipe_;
    rs2::pipeline_profile pipeProfile_;
    rs2::device rsDevice_;
    rs2_intrinsics rsDepthIntrinsics_{};
    rs2_intrinsics rsRGBIntrinsics_{};
    rs2::sensor rsStereoSensor_;
    rs2::sensor rsRGBSensor_;
    std::vector<VideoMode> color_modes_;
    std::vector<int> color_fps_;
    std::vector<VideoMode> depth_modes_;
    std::vector<int> depth_fps_;
    rs2::decimation_filter rs_dec_filter_;
    rs2::threshold_filter rs_thr_filter_;
    rs2::spatial_filter rs_spat_filter_;
    rs2::temporal_filter rs_temp_filter_;
    rs2::hole_filling_filter rs_hole_fill_filter_;
    std::unique_ptr<rs2::disparity_transform> rs_depth_to_disparity_;
    std::unique_ptr<rs2::disparity_transform> rs_disparity_to_depth_;
    rs2::frame_queue original_data_;
    rs2::frame_queue filtered_data_;
    int color_mode_idx_;
    int depth_mode_idx_;
    int preset_idx_;
    int laser_power_;
    bool is_valid_color_mode_;
    bool is_valid_depth_mode_;
    bool should_init_;
    bool should_set_params_;
    int color_fps_idx_;
    int depth_fps_idx_;
    int rs_cam_idx_;
    ImageControls depth_controls_{};
    ImageControls color_controls_{};
    std::string rs_cam_serial_;
    int decimation_amount_;
    int thresh_min_;
    int thresh_max_;
    int spatial_mag_;
    float spatial_alpha_;
    int spatial_delta_;
    int spatial_hole_fill_;
    float temporal_alpha_;
    int temporal_delta_;
    int temporal_persistency_;
    int hole_fill_option_;
    bool enable_color_;
    bool enable_depth_;
    bool init_once_;
    bool cam_is_init_;
    bool first_list_refresh_;
    bool enable_decimation_filter_;
    bool enable_threshold_filter_;
    bool enable_spatial_filter_;
    bool enable_temporal_filter_;
    bool enable_hole_fill_filter_;
    bool align_color_to_depth_;
    bool align_depth_to_color_;
};

EXPORT_PLUGIN( RealsenseCamera )

}  // namespace DSPatch::DSPatchables

#endif //FLOWCV_REALSENSE_CAMERA_HPP_
