#pragma once

#include<librealsense2/rs.hpp>
#include<opencv2/opencv.hpp>
#include <iostream>

class RealSenseCam1
{
public:
    //specify the resolution and frame rate
    RealSenseCam1(int width, int height, int fps, const std::string& serial );
    ~RealSenseCam1();

    //get a color image
    bool grabColor(cv::Mat &color_out, double &timestamp_ms);


private:
   //creat a pipeline, this serves as a top-level API for streaming and processing frames.
   rs2::pipeline pipe_;
   rs2::config cfg_;

   int width_;
   int height_;
   int fps_;
    

};




RealSenseCam1::RealSenseCam1(int width, int height, int fps, const std::string& serial):width_(width), height_(height), fps_(fps)
{
    try
    {   
        cfg_.enable_device(serial);
        //RGB tell camera what stream and format i want to use
        cfg_.enable_stream(RS2_STREAM_COLOR, width_, height_, RS2_FORMAT_BGR8, fps_);
        //start camera
        pipe_.start(cfg_);

        std::cout << "[RealSenseCamera1] Started D435i color stream: "
                  << width_ << "x" << height_ << " @ " << fps_ << " FPS\n";
    }
    catch(const rs2::error& e){
        std::cerr << "RealSense error in ctor: " << e.what() << std::endl;
        throw;
    }
    catch(const std::exception& e){
        std::cerr << "Std exception in actor: " << e.what() << std::endl;
        throw;
    }
}


RealSenseCam1::~RealSenseCam1()
{
    try
    {
        pipe_.stop();
        std::cout << "[RealSenseCamera1] Pipeline stopped.\n ";
    }
    catch(...) {}

}

bool RealSenseCam1::grabColor(cv::Mat &color_out, double &timestamp_ms)
{
    try
    {
        rs2::frameset frames = pipe_.wait_for_frames(); // wait and get the frameset
        rs2::video_frame color_frame = frames.get_color_frame();  //extract the required color frames
        if(!color_frame){
            return false;
        }
    
    //RealSense timestamp
    timestamp_ms = color_frame.get_timestamp();

    // store the camera data in thr temporary container color_tmp, then copy it to color_out
    cv::Mat color_tmp(cv::Size(width_, height_),  CV_8UC3, (void*)color_frame.get_data(),
                          cv::Mat::AUTO_STEP);
    color_tmp.copyTo(color_out);

    return true;

     }


    catch (const rs2::error &e) {
        std::cerr << "RealSense error in grabColor: " << e.what() << std::endl;
        return false;
    }
    catch (const std::exception &e) {
        std::cerr << "Std exception in grabColor: " << e.what() << std::endl;
        return false;
    }
};
