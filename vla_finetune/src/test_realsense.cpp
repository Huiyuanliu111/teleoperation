#include <iostream>
#include "RealSenseCam1.cpp"   
int main() {
    std::cout << "=== RealSenseCam1 Test Start ===\n";


    RealSenseCam1 cam(640, 480, 30, "233622072733");

    cv::Mat color; //cv::mat: a comtainer for an image
    double timestamp_ms = 0.0;

    while (true) {
        bool ok = cam.grabColor(color, timestamp_ms);
        if (!ok) {
            std::cerr << "Failed to grab frame!\n";
            continue;
        }

        // show the image
        cv::imshow("RealSense Color", color);

        
        char key = (char)cv::waitKey(1);
        if (key == 'q' || key == 'Q') {
            break;
        }
    }

    std::cout << "=== RealSenseCam1 Test End ===\n";
    return 0;
}
