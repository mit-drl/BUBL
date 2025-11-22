import argparse
import sys
import time
import cv2
import numpy as np


def make_detector():
    if not hasattr(cv2, "aruco"):
        print("cv2.aruco missing. Install:")
        print("    pip install opencv-contrib-python")
        sys.exit(1)

    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)

    try:
        parameters = aruco.DetectorParameters()
    except AttributeError:
        parameters = aruco.DetectorParameters_create()

    # Light small-tag tuning (same as your script)
    parameters.minMarkerPerimeterRate = 0.01
    parameters.minCornerDistanceRate = 0.02
    if hasattr(aruco, "CORNER_REFINE_APRILTAG"):
        parameters.cornerRefinementMethod = aruco.CORNER_REFINE_APRILTAG
    parameters.perspectiveRemovePixelPerCell = 8

    return aruco, dictionary, parameters


def main():
    parser = argparse.ArgumentParser(
        description="Real-time AprilTag (tag36h11) tracking from webcam (e.g., GoPro)."
    )
    parser.add_argument(
        "--camera_index",
        type=int,
        default=0,
        help="Index of the webcam (default: 0). "
             "Try 1, 2, ... if your GoPro is on a different index."
    )
    parser.add_argument(
        "--width",
        type=int,
        default=1280,
        help="Requested capture width (default: 1280)."
    )
    parser.add_argument(
        "--height",
        type=int,
        default=720,
        help="Requested capture height (default: 720)."
    )
    parser.add_argument(
        "--scale_display",
        type=float,
        default=0.7,
        help="Scale factor for the display window (does NOT affect detection). "
             "1.0 = full size, <1 shrinks the window."
    )
    args = parser.parse_args()

    # Open webcam (GoPro should appear as a camera device)
    cap = cv2.VideoCapture(args.camera_index)
    if not cap.isOpened():
        print(f"Could not open camera index {args.camera_index}")
        sys.exit(1)

    # Try to set resolution (camera may not honor it exactly)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    aruco, dictionary, parameters = make_detector()

    win_name = "AprilTag Overlay (Real-time Webcam)"
    cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)

    print("Press 'q' in the video window to quit.\n")

    t_prev = time.time()

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame from camera.")
            break

        # Full-resolution detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        t0 = time.time()
        corners, ids, _ = aruco.detectMarkers(gray, dictionary, parameters=parameters)
        t1 = time.time()
        detect_dt = t1 - t0

        # Build display frame (optionally scaled down for smaller window)
        if args.scale_display != 1.0:
            frame_disp = cv2.resize(
                frame,
                None,
                fx=args.scale_display,
                fy=args.scale_display,
                interpolation=cv2.INTER_AREA
            )
            scale = args.scale_display
        else:
            frame_disp = frame.copy()
            scale = 1.0

        # Draw detections
        if ids is not None and len(corners) > 0:
            ids_flat = ids.flatten().astype(int)
            for marker_corners, tag_id in zip(corners, ids_flat):
                pts = marker_corners.reshape((4, 2)).astype(np.float32)
                # scale to display size if needed
                pts_scaled = np.rint(pts * scale).astype(int)

                # Borders
                for i in range(4):
                    pt1 = tuple(pts_scaled[i])
                    pt2 = tuple(pts_scaled[(i + 1) % 4])
                    cv2.line(frame_disp, pt1, pt2, (0, 255, 0), 2)

                # Center
                center = np.mean(pts_scaled, axis=0).astype(int)
                cv2.circle(frame_disp, tuple(center), 4, (0, 0, 255), -1)

                # ID text
                text = f"id={tag_id}"
                cv2.putText(
                    frame_disp,
                    text,
                    (pts_scaled[0][0], pts_scaled[0][1] - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA
                )

        # Processing FPS overlay (how fast this loop is running)
        t_now = time.time()
        dt_frame = t_now - t_prev
        t_prev = t_now
        if dt_frame > 0:
            proc_fps = 1.0 / dt_frame
            cv2.putText(
                frame_disp,
                f"proc FPS: {proc_fps:4.1f}   detect: {detect_dt*1000:5.1f} ms",
                (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
                cv2.LINE_AA
            )

        cv2.imshow(win_name, frame_disp)

        # Wait just 1 ms so we run as fast as possible
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
