import argparse
import sys
import time
import cv2
import numpy as np


def pick_video_file():
    try:
        import tkinter as tk
        from tkinter import filedialog
        root = tk.Tk()
        root.withdraw()
        path = filedialog.askopenfilename(
            title="Select video file",
            filetypes=[("Video files", "*.mp4 *.avi *.mov *.mkv"), ("All files", "*.*")]
        )
        root.destroy()
        return path
    except:
        return None


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

    # Light small-tag tuning
    parameters.minMarkerPerimeterRate = 0.01
    parameters.minCornerDistanceRate = 0.02
    if hasattr(aruco, "CORNER_REFINE_APRILTAG"):
        parameters.cornerRefinementMethod = aruco.CORNER_REFINE_APRILTAG
    parameters.perspectiveRemovePixelPerCell = 8

    return aruco, dictionary, parameters


def main():
    parser = argparse.ArgumentParser(
        description="Full-frame AprilTag detect every ~2 FPS, measure detect time."
    )
    parser.add_argument("--video", type=str, default=None,
                        help="Path to video. If omitted, file dialog opens.")
    parser.add_argument("--detections_per_second", type=float, default=2.0,
                        help="How many times per second to run full detection.")
    args = parser.parse_args()

    video_path = args.video
    if video_path is None:
        video_path = pick_video_file()
        if not video_path:
            print("No video selected.")
            sys.exit(0)

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print("Could not open video:", video_path)
        sys.exit(1)

    # Read video FPS
    fps = cap.get(cv2.CAP_PROP_FPS)
    if fps <= 0:
        fps = 30.0
    frame_delay_ms = max(1, int(1000.0 / fps))

    detect_interval = max(1, int(round(fps / args.detections_per_second)))

    aruco, dictionary, parameters = make_detector()

    win_name = "AprilTag Overlay (2 FPS Detection, Timed)"
    cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)

    print("Video FPS:", fps)
    print("Target detections/sec:", args.detections_per_second)
    print("Detection interval (frames):", detect_interval)
    print("Press 'q' to quit.\n")

    frame_idx = 0
    t_prev = time.time()

    last_corners = None
    last_ids = None

    # Detection time statistics
    detect_times = []

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame_idx += 1
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        run_detection = (frame_idx == 1) or (frame_idx % detect_interval == 0)

        if run_detection:
            t0 = time.time()
            corners, ids, _ = aruco.detectMarkers(gray, dictionary, parameters=parameters)
            t1 = time.time()

            dt = t1 - t0
            detect_times.append(dt)
            avg_dt = sum(detect_times) / len(detect_times)

            print(f"[Frame {frame_idx}] Detection time: {dt*1000:.1f} ms    "
                  f"(avg: {avg_dt*1000:.1f} ms)")

            if ids is not None and len(corners) > 0:
                last_corners = [c.reshape((4, 2)).astype(np.float32) for c in corners]
                last_ids = ids.flatten().astype(int)
            else:
                last_corners = None
                last_ids = None

        # Draw previous detection
        frame_disp = frame.copy()

        if last_corners is not None and last_ids is not None:
            for pts, tag_id in zip(last_corners, last_ids):
                pts_int = np.rint(pts).astype(int)

                # Borders
                for i in range(4):
                    pt1 = tuple(pts_int[i])
                    pt2 = tuple(pts_int[(i + 1) % 4])
                    cv2.line(frame_disp, pt1, pt2, (0, 255, 0), 2)

                # Center
                center = np.mean(pts_int, axis=0).astype(int)
                cv2.circle(frame_disp, tuple(center), 4, (0, 0, 255), -1)

                # ID text
                text = f"id={tag_id}"
                cv2.putText(frame_disp, text,
                            (pts_int[0][0], pts_int[0][1] - 5),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 255, 255), 2)

        # FPS overlay for processing speed
        t_now = time.time()
        dt_frame = t_now - t_prev
        t_prev = t_now
        if dt_frame > 0:
            proc_fps = 1.0 / dt_frame
            cv2.putText(frame_disp, f"proc FPS: {proc_fps:.1f}",
                        (10, 20), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (255, 255, 255), 2)

        cv2.imshow(win_name, frame_disp)

        if cv2.waitKey(frame_delay_ms) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

    # Print summary
    if detect_times:
        print("\n===== Detection Timing Summary =====")
        print(f"Detections run: {len(detect_times)}")
        print(f"Average detect time: {np.mean(detect_times)*1000:.1f} ms")
        print(f"Median detect time:  {np.median(detect_times)*1000:.1f} ms")
        print(f"Fastest detect:      {np.min(detect_times)*1000:.1f} ms")
        print(f"Slowest detect:      {np.max(detect_times)*1000:.1f} ms")
        print("====================================\n")
    else:
        print("No detections were performed.")


if __name__ == "__main__":
    main()
