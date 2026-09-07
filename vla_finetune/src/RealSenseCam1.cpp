#pragma once

#include <librealsense2/rs.hpp>
#include <opencv2/core.hpp>

#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>

struct RealSenseRgbdFrame
{
    cv::Mat color_bgr;
    cv::Mat depth_z16_aligned_to_color;
    double color_timestamp_ms = 0.0;
    double depth_timestamp_ms = 0.0;
    uint64_t color_frame_number = 0;
    uint64_t depth_frame_number = 0;
    int64_t host_timestamp_ns = 0;
};

class RealSenseCam1
{
public:
    RealSenseCam1(int width, int height, int fps, const std::string &serial);
    ~RealSenseCam1();

    bool grabRgbd(RealSenseRgbdFrame &frame);

    const rs2_intrinsics &colorIntrinsics() const { return color_intrinsics_; }
    const rs2_intrinsics &depthIntrinsics() const { return depth_intrinsics_; }
    const rs2_extrinsics &depthToColorExtrinsics() const { return depth_to_color_; }
    float depthScaleMeters() const { return depth_scale_m_; }
    const std::string &serial() const { return serial_; }

private:
    rs2::pipeline pipe_;
    rs2::config cfg_;
    rs2::align align_to_color_;
    std::string serial_;
    int width_;
    int height_;
    int fps_;
    float depth_scale_m_ = 0.001f;
    rs2_intrinsics color_intrinsics_{};
    rs2_intrinsics depth_intrinsics_{};
    rs2_extrinsics depth_to_color_{};
};

RealSenseCam1::RealSenseCam1(
    int width, int height, int fps, const std::string &serial)
    : align_to_color_(RS2_STREAM_COLOR), serial_(serial), width_(width),
      height_(height), fps_(fps)
{
    try
    {
        cfg_.enable_device(serial_);
        cfg_.enable_stream(
            RS2_STREAM_COLOR, width_, height_, RS2_FORMAT_BGR8, fps_);
        cfg_.enable_stream(
            RS2_STREAM_DEPTH, width_, height_, RS2_FORMAT_Z16, fps_);
        rs2::pipeline_profile profile = pipe_.start(cfg_);

        const rs2::video_stream_profile color_profile =
            profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
        const rs2::video_stream_profile depth_profile =
            profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
        color_intrinsics_ = color_profile.get_intrinsics();
        depth_intrinsics_ = depth_profile.get_intrinsics();
        depth_to_color_ = depth_profile.get_extrinsics_to(color_profile);
        depth_scale_m_ = profile.get_device()
                             .first<rs2::depth_sensor>()
                             .get_depth_scale();

        std::cout << "[RealSense] Started RGB-D serial " << serial_ << ": "
                  << width_ << "x" << height_ << " @ " << fps_
                  << " FPS, depth_scale=" << depth_scale_m_ << " m/unit\n";
    }
    catch (const rs2::error &e)
    {
        std::cerr << "RealSense error in ctor for " << serial_ << ": "
                  << e.what() << std::endl;
        throw;
    }
}

RealSenseCam1::~RealSenseCam1()
{
    try
    {
        pipe_.stop();
        std::cout << "[RealSense] Pipeline stopped for " << serial_ << "\n";
    }
    catch (...)
    {
    }
}

bool RealSenseCam1::grabRgbd(RealSenseRgbdFrame &output)
{
    try
    {
        rs2::frameset frames = pipe_.wait_for_frames();
        // Timestamp receipt before alignment/copies so per-camera processing
        // jitter is not folded into the inter-camera synchronization error.
        const int64_t host_timestamp_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count();
        rs2::frameset aligned = align_to_color_.process(frames);
        const rs2::video_frame color = aligned.get_color_frame();
        const rs2::depth_frame depth = aligned.get_depth_frame();
        if (!color || !depth)
        {
            return false;
        }

        output.host_timestamp_ns = host_timestamp_ns;
        output.color_timestamp_ms = color.get_timestamp();
        output.depth_timestamp_ms = depth.get_timestamp();
        output.color_frame_number = color.get_frame_number();
        output.depth_frame_number = depth.get_frame_number();

        cv::Mat color_view(
            cv::Size(width_, height_), CV_8UC3,
            const_cast<void *>(color.get_data()), cv::Mat::AUTO_STEP);
        cv::Mat depth_view(
            cv::Size(width_, height_), CV_16UC1,
            const_cast<void *>(depth.get_data()), cv::Mat::AUTO_STEP);
        color_view.copyTo(output.color_bgr);
        depth_view.copyTo(output.depth_z16_aligned_to_color);
        return true;
    }
    catch (const rs2::error &e)
    {
        std::cerr << "RealSense error in grabRgbd for " << serial_ << ": "
                  << e.what() << std::endl;
        return false;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Exception in grabRgbd for " << serial_ << ": "
                  << e.what() << std::endl;
        return false;
    }
}
