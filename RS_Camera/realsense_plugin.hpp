//
// FlowCV Plugin - Realsense Camera
// Written By Richard Wardlow
//

#ifndef FLOWCV_REALSENSE_PLUGIN_HPP_
#define FLOWCV_REALSENSE_PLUGIN_HPP_
#include <DSPatch.h>
#include "FlowCV_Types.hpp"
#include "realsense_camera.hpp"
#include <iostream>
#include <vector>
#include <string>


namespace DSPatch::DSPatchables
{
namespace internal
{
class RealsenseCamera;
}

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

  private:
    std::unique_ptr<internal::RealsenseCamera> p;
    std::mutex io_mutex_;
    std::unique_ptr<realsense_camera> camera_;
    int alignment_mode_;
    int selected_camera_idx_;
    int color_cfg_idx_;
    int color_fps_idx_;
    int color_fps_;
    int depth_cfg_idx_;
    int depth_fps_idx_;
    int depth_fps_;
    bool enable_color_;
    bool enable_depth_;
};

EXPORT_PLUGIN( RealsenseCamera )

}  // namespace DSPatch::DSPatchables

#endif //FLOWCV_REALSENSE_PLUGIN_HPP_
