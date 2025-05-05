import cv2
import numpy as np
import pyapriltags as apriltag

def readApriltag(tagID,dir):
    #----------------------------------------------------------------------
    # 1. Load camera calibration data
    #----------------------------------------------------------------------
    calibration_data = np.load('phases\camera_calibration_live_front.npz')  # adjust filename
    camera_matrix = calibration_data['camera_matrix']  # shape (3, 3)
    dist_coeffs = calibration_data['dist_coeffs']      # shape (n,) typically (5,) or (8,)

    # Extract focal lengths and principal point from the camera matrix
    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]


    at_detector = apriltag.Detector(
        families='tag36h11',  # or 'tag25h9', etc.
        nthreads=1,
        quad_decimate=1.0,
        quad_sigma=0.0,
        refine_edges=1,
        decode_sharpening=0.25,
        debug=0
    )

    # The length of one side of the tag in meters
    TAG_SIZE = 0.10  # 10 cm

    #----------------------------------------------------------------------
    # 3. Initialize Webcam
    #----------------------------------------------------------------------
    cap = cv2.VideoCapture("/dev/video10")
    if dir:
        cap = cv2.VideoCapture("/dev/video4")


    #cap = cv2.VideoCapture("/dev/video10") # OR "/dev/video4" (side camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    cap.set(cv2.CAP_PROP_FOCUS, 0)
    cap.set(cv2.CAP_PROP_FOURCC ,cv2.VideoWriter_fourcc('M', 'J', 'P', 'G') )

    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    print("Press 'q' to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to read from the webcam.")
            break

        undistorted = frame # This turns off the undistortion

        # Convert to grayscale
        gray = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)

        results = at_detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=[fx, fy, cx, cy],  # from your calibration
            tag_size=TAG_SIZE
        )

        #------------------------------------------------------------------
        # 6. Process each detection
        #------------------------------------------------------------------
        for r in results:
            tag_id = r.tag_id
            if (tag_id == tagID):
                corners = r.corners.astype(int)

                #----------------------------------------
                # 6b. Draw the detection on the image
                #----------------------------------------
                # Draw the outline of the tag
                for i in range(4):
                    cv2.line(
                        undistorted,
                        tuple(corners[i]),
                        tuple(corners[(i+1) % 4]),
                        (0, 255, 0),
                        2
                    )

                # Draw the tag ID near the center
                center_xy = (int(r.center[0]), int(r.center[1]))
                cv2.putText(undistorted, f"ID: {tag_id}", center_xy,
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

                #----------------------------------------
                # 6c. Get Pose (R, t)
                #----------------------------------------
                R = r.pose_R  # 3×3 rotation matrix
                t = r.pose_t  # 3×1 translation vector

                rot_vec, _ = cv2.Rodrigues(R)     # from rotation matrix to rotation vector
                rot_deg = np.degrees(rot_vec)     # convert to degrees for display if desired

                # print(f"Detected Tag ID {tag_id}:")
                # print(f"  Translation (x, y, z) [m]: {t.ravel()}")

                cv2.putText(undistorted, "X: " + str(round(float(t[0]),2)) + ", Y: " + str(round(float(t[1]),2)) + ", Z: " + str(round(float(t[2]),2)), corners[0], cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                return (round(float(t[0]),2), round(float(t[2]),2)) # Returns (x, z)

        cv2.imshow('AprilTag Detection', undistorted)

        return (9999,9999)
        # Press 'q' to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
