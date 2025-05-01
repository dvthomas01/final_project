
#include <librealsense2/rs.hpp>
#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>

volatile sig_atomic_t keep_running = 1;

void signal_handler(int /*signum*/) {
    keep_running = 0;
}

int main() {
    //start process for camera
    std::signal(SIGINT, signal_handler);
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    pipe.start(cfg);
    std::cout << "starting & streaming now\n";

    while (keep_running) {
        // waiting for a new frame
        auto frames = pipe.wait_for_frames();
        auto depth  = frames.get_depth_frame();
        int w = depth.get_width();
        int h = depth.get_height();
        int x = w / 2;
        int y = h / 2;
        // get the distance at the center of the frame
        float dist = depth.get_distance(x, y);
        std::cout << "\rdistance at (" << x << "," << y << "): "
                  << dist << " m        " << std::flush;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    pipe.stop();
    std::cout << "\nStopped.\n";
    return 0;
}
