#include <iostream>
#include <string>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "RealSenseCam1.cpp"

namespace
{
void list_devices()
{
    rs2::context context;
    const rs2::device_list devices = context.query_devices();
    std::cout << "Detected " << devices.size() << " RealSense device(s)\n";
    for (size_t index = 0; index < devices.size(); ++index)
    {
        const rs2::device device = devices[index];
        const char *name = device.supports(RS2_CAMERA_INFO_NAME)
                               ? device.get_info(RS2_CAMERA_INFO_NAME)
                               : "unknown";
        const char *serial = device.supports(RS2_CAMERA_INFO_SERIAL_NUMBER)
                                 ? device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)
                                 : "unknown";
        const char *port = device.supports(RS2_CAMERA_INFO_PHYSICAL_PORT)
                               ? device.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT)
                               : "unknown";
        std::cout << "[" << index << "] serial=" << serial << " name=" << name
                  << " physical_port=" << port << '\n';
    }
}
} // namespace

int main(int argc, char **argv)
{
    if (argc == 1 || (argc == 2 && std::string(argv[1]) == "--list"))
    {
        list_devices();
        return 0;
    }
    if (argc != 3 || std::string(argv[1]) != "--preview")
    {
        std::cerr << "Usage: " << argv[0] << " --list\n"
                  << "       " << argv[0] << " --preview SERIAL\n";
        return 2;
    }

    const std::string serial = argv[2];
    std::cout << "=== RGB-D preview for serial " << serial << " ===\n";
    RealSenseCam1 cam(640, 480, 30, serial);

    while (true) {
        RealSenseRgbdFrame frame;
        bool ok = cam.grabRgbd(frame);
        if (!ok) {
            std::cerr << "Failed to grab frame!\n";
            continue;
        }

        // show the image
        cv::imshow("RealSense Color", frame.color_bgr);
        cv::Mat depth_preview;
        frame.depth_z16_aligned_to_color.convertTo(
            depth_preview, CV_8U, 255.0 / 3000.0);
        cv::applyColorMap(depth_preview, depth_preview, cv::COLORMAP_TURBO);
        cv::imshow("Aligned Depth", depth_preview);

        
        char key = (char)cv::waitKey(1);
        if (key == 'q' || key == 'Q') {
            break;
        }
    }

    std::cout << "=== RealSense RGB-D test ended ===\n";
    return 0;
}
