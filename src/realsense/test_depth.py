import pyrealsense2 as rs
import time

#camera start process
pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipe.start(cfg)

try: 
    while True:
        #here we get the last depth frame
        depth_frame = pipe.wait_for_frames().get_depth_frame()

        x = depth_frame.get_width() //2
        y = depth_frame.get_height()//2

        distance = depth_frame.get_distance(x,y)
        print(f"distance at pixel ({x},{y}): {distance:.2f}m away")
        time.sleep(1.0)
except KeyboardInterrupt:
    pipe.stop()
    print("stopped")