import imageio.v3 as iio
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
import cv2


from gaze_lib.detector import GazeDetector  # adjust path to your module

# camera parameters
fx, fy = 374.29986572265625, 374.93951416015625
cx, cy = 259.0893249511719, 221.61053466796875
depth_scale = 1000.0 
depth_encoding = "16UC1"

# Open3D camera intrinsic
intrinsic = o3d.camera.PinholeCameraIntrinsic()
intrinsic.set_intrinsics(width=512, height=424, fx=fx, fy=fy, cx=cx, cy=cy)

# gaze detector initialization
t0 = 0.0  
camera_info = {"fx": fx, "fy": fy, "cx": cx, "cy": cy}
detector = GazeDetector(t0, camera_info, depth_encoding, depth_scale)

rgb_reader = iio.imiter("/home/yasmine/Documents/gaze_detection/pointing_gesture_recognition/gaze_detector/dataset/data/rgb.avi", plugin="pyav")
depth_reader = iio.imiter("/home/yasmine/Documents/gaze_detection/pointing_gesture_recognition/gaze_detector/dataset/data/depth.avi", plugin="pyav")

for i, (rgb, depth_frame) in enumerate(zip(rgb_reader, depth_reader)):
    print(f"\n=== Frame {i} ===")

    G = depth_frame[..., 1]
    B = depth_frame[..., 2]
    depth16 = (G.astype(np.uint16) << 8) + B.astype(np.uint16)
    depth_m = depth16.astype(np.float32) / depth_scale

    timestamp = i * 0.033  # fake timestamp ~30 fps
    print(f"Timestamp: {timestamp:.3f} seconds")

    try :
        # img = cv2.imread("/home/yasmine/Documents/gaze_detection/pointing_gesture_recognition/gaze_detector/test_image.jpg", cv2.IMREAD_COLOR)
        # rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        h, w, _ = rgb.shape
        zoom_factor = 2  # 2x zoom par exemple
        center = (w // 2, h // 2)
        cropped = rgb[center[1] - h//(2*zoom_factor) : center[1] + h//(2*zoom_factor),
                            center[0] - w//(2*zoom_factor) : center[0] + w//(2*zoom_factor)]
        rgb = cv2.resize(cropped, (w, h))
        gaze_keypoints, eye_detected, head_coord_sys, dx, dy = detector.detect_eye_gaze(rgb, depth_m, timestamp)

    except Exception as e:
        print(f"Error during gaze detection: {e}")
        eye_detected = False
        gaze_keypoints = None
        continue
    
    # convert depth to cloud
    depth_o3d = o3d.geometry.Image(depth_m)
    pcd = o3d.geometry.PointCloud.create_from_depth_image(
        depth_o3d, intrinsic, depth_scale=1.0, depth_trunc=5.0, stride=1
    )
    pcd = pcd.remove_non_finite_points()
    pcd = pcd.voxel_down_sample(0.005)


    if not eye_detected or gaze_keypoints is None:
        print("No eye detected.")
        o3d.visualization.draw_geometries([pcd])
        continue

    eye_pos = np.array(gaze_keypoints["right_eye_center"])  # (x, y, z)
    if head_coord_sys is None:
        print("No head coordinate system.")
        continue
    head_rotation = np.array(head_coord_sys)

    gaze_dir = head_rotation @ np.array([0, 0, 1])
    gaze_end = eye_pos + gaze_dir * 0.5 

    # plot gaze
    gaze_line = o3d.geometry.LineSet()
    gaze_line.points = o3d.utility.Vector3dVector([eye_pos, gaze_end])
    gaze_line.lines = o3d.utility.Vector2iVector([[0, 1]])
    gaze_line.colors = o3d.utility.Vector3dVector([[1, 0, 0]])  # red

    o3d.visualization.draw_geometries([pcd, gaze_line])

    print("i =", i)
    if i >= 10:
        break
