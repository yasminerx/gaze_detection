import imageio.v3 as iio
import numpy as np
import io
import matplotlib.pyplot as plt  # For visualization
import open3d as o3d
fx, fy = 374.29986572265625, 374.93951416015625
cx, cy = 259.0893249511719, 221.61053466796875

intrinsic = o3d.camera.PinholeCameraIntrinsic()
intrinsic.set_intrinsics(width=512, height=424, fx=fx, fy=fy, cx=cx, cy=cy)

# === Option 1: Read directly from disk ===
video_path = "/home/yasmine/Documents/v4r/gaze_detection/pointing_gesture_recognition/gaze_detector/dataset/data/depth.avi"  # Replace with actual path
reader = iio.imiter(video_path, plugin="pyav")  # or plugin="ffmpeg"

# === Option 2: Read from BytesIO (if needed) ===
# with open(video_path, "rb") as f:
#     video_data = io.BytesIO(f.read())
#     reader = iio.imiter(video_data, extension=".avi", plugin="pyav")

# === Iterate over frames ===
for i, frame in enumerate(reader):
    print(f"\n=== Frame {i} ===")
    print(f"Shape: {frame.shape}, Dtype: {frame.dtype}")

    # Extract channels
    R = frame[..., 0]
    G = frame[..., 1]
    B = frame[..., 2]

    print(f"R unique: {np.unique(R)}")  # should be [0]
    print(f"G min/max: {G.min()} / {G.max()}")
    print(f"B min/max: {B.min()} / {B.max()}")

    # Reconstruct 16-bit depth
    depth16 = (G.astype(np.uint16) << 8) + B.astype(np.uint16)
    #depth16 = B.astype(np.uint16)

    print(f"Depth16 min/max: {depth16.min()} / {depth16.max()}, dtype={depth16.dtype}")

    # Convert to meters
    depth_m = depth16.astype(np.float32) / 1000.0  # now in meters

    # Convert to Open3D image
    depth_o3d = o3d.geometry.Image(depth_m)

    # === Generate Point Cloud ===
    pcd = o3d.geometry.PointCloud.create_from_depth_image(
        depth_o3d,
        intrinsic,
        depth_scale=1.0,  # already in meters
        depth_trunc=5.0,  # clip depth at 5m
        stride=1,
    )

    # Optionally remove NaNs or zero-points
    pcd = pcd.remove_non_finite_points()
    pcd = pcd.voxel_down_sample(voxel_size=0.005)

    # === Visualize ===
    o3d.visualization.draw_geometries([pcd], window_name="Recovered Depth Point Cloud")

    # Display
    plt.imshow(depth16, cmap='gray', vmin=depth16.min(), vmax=depth16.max())
    plt.title(f"Reconstructed 16-bit Depth — Frame {i}")
    plt.colorbar()
    plt.axis("off")
    plt.pause(0.01)
    plt.close()

    if i >= 10:
        break
