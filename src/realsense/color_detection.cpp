// detect_tape_color.cpp
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <map>
#include <vector>
#include <string>

int main() {
    //  start camera
    rs2::pipeline pipe;
    rs2::config  cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    pipe.start(cfg);
    std::cout << "🎥  Streaming color. Press ESC to quit.\n";

    // hsv ranges
    std::map<std::string,std::vector<std::pair<cv::Scalar,cv::Scalar>>> color_ranges = {
        { "red",    { { {  0,120, 70 }, { 10,255,255 } },
                      { {170,120, 70 }, {180,255,255 } } } },
        { "green",  { { { 36,100,100 }, { 86,255,255 } } } },
        { "blue",   { { { 94, 80,  2 }, {126,255,255 } } } },
        { "yellow", { { { 15,100,100 }, { 35,255,255 } } } }
    };

    while (true) {
        
        rs2::frameset frames = pipe.wait_for_frames();
        rs2::video_frame color = frames.get_color_frame();
        if (!color) continue;

        // wrap frame data in an openCV Mat
        int w = color.get_width(), h = color.get_height();
        cv::Mat img(cv::Size(w,h), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);

        cv::Mat hsv;
        cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

        // color detection
        std::string best_color = "none";
        int best_count = 0;
        for (auto &kv : color_ranges) {
            const auto &name   = kv.first;
            const auto &ranges = kv.second;

            cv::Mat mask;
            for (size_t i = 0; i < ranges.size(); ++i) {
                cv::Mat m;
                cv::inRange(hsv, ranges[i].first, ranges[i].second, m);
                if (i == 0)      mask = m;
                else             cv::bitwise_or(mask, m, mask);
            }

            int count = cv::countNonZero(mask);
            if (count > best_count) {
                best_count = count;
                best_color = name;
            }
        }

        //  tape threshold
        std::string label = (best_count > 700)
            ? "Tape color: " + best_color
            : "No tape detected";

        // Overlay & show
        cv::putText(img, label, {10,30},
                    cv::FONT_HERSHEY_SIMPLEX, 1.0,
                    {0,255,0}, 2);
        cv::imshow("Tape-Color Detector", img);

        // Exit on ESC
        if ((cv::waitKey(1) & 0xFF) == 27) break;
    }


    pipe.stop();
    cv::destroyAllWindows();
    return 0;
}
