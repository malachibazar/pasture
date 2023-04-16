import cv2
import numpy as np
import os
import matplotlib.pyplot as plt
import g2o

fx = 1_300  # Focal length in x-direction (in pixels)
fy = 1_400  # Focal length in y-direction (in pixels)
cx = 1_640  # Principal point x-coordinate (in pixels)
cy = 1_232  # Principal point y-coordinate (in pixels)


# Feature extraction and matching
def extract_features(img, detector):
    keypoints, descriptors = detector.detectAndCompute(img, None)
    return keypoints, descriptors


def match_features(descriptors1, descriptors2, matcher):
    matches = matcher.match(descriptors1, descriptors2)
    return matches


# Motion estimation
def estimate_motion(matches, keypoints1, keypoints2, intrinsic_matrix):
    src_pts = np.float32([keypoints1[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
    dst_pts = np.float32([keypoints2[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)

    E, mask = cv2.findEssentialMat(src_pts, dst_pts, intrinsic_matrix)
    _, R, t, _ = cv2.recoverPose(E, src_pts, dst_pts, intrinsic_matrix)

    return R, t


# Triangulation
def triangulate_points(matches, keypoints1, keypoints2, R, t, intrinsic_matrix):
    src_pts = np.float32([keypoints1[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
    dst_pts = np.float32([keypoints2[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)

    P1 = np.hstack((np.eye(3), np.zeros((3, 1))))
    P1 = intrinsic_matrix @ P1

    P2 = np.hstack((R, t))
    P2 = intrinsic_matrix @ P2

    points_3D_homogeneous = cv2.triangulatePoints(P1, P2, src_pts.T, dst_pts.T)
    points_3D = points_3D_homogeneous[:3, :] / points_3D_homogeneous[3, :]

    return points_3D.T


# Loop closure detection
def detect_loop_closure(
    current_keypoints, current_descriptors, keyframes, matcher, min_matches=20
):
    loop_closure = None
    best_num_matches = -1

    for i, (keypoints, descriptors) in enumerate(keyframes):
        matches = match_features(current_descriptors, descriptors, matcher)

        if len(matches) > best_num_matches and len(matches) >= min_matches:
            src_pts = np.float32(
                [current_keypoints[m.queryIdx].pt for m in matches]
            ).reshape(-1, 1, 2)
            dst_pts = np.float32([keypoints[m.trainIdx].pt for m in matches]).reshape(
                -1, 1, 2
            )

            # Estimate the homography matrix and check geometric consistency
            H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)

            if np.sum(mask) >= min_matches:
                best_num_matches = len(matches)
                loop_closure = i

    return loop_closure


def optimize_bundle_adjustment(poses, points_3D, observations, local=False):
    optimizer = g2o.SparseOptimizer()
    solver = g2o.BlockSolverSE3(g2o.LinearSolverEigenSE3())
    algorithm = g2o.OptimizationAlgorithmLevenberg(solver)
    optimizer.set_algorithm(algorithm)

    # Add camera poses
    pose_vertices = []
    for i, pose in enumerate(poses):
        vertex = g2o.VertexSE3Expmap()
        vertex.set_estimate(g2o.SE3Quat(pose[:3, :3], pose[:3, 3]))
        vertex.set_id(i)
        if i == 0 and not local:
            vertex.set_fixed(True)  # Fix the first pose for global bundle adjustment
        optimizer.add_vertex(vertex)
        pose_vertices.append(vertex)

    # Add 3D points
    point_vertices = []
    for i, point_3D in enumerate(points_3D):
        vertex = g2o.VertexSBAPointXYZ()
        vertex.set_estimate(point_3D)
        vertex.set_id(len(poses) + i)
        vertex.set_marginalized(True)
        optimizer.add_vertex(vertex)
        point_vertices.append(vertex)

    # Add observations
    for point_3D_idx, pose_idx, observation in observations:
        edge = g2o.EdgeProjectXYZ2UV()
        edge.set_vertex(0, point_vertices[point_3D_idx])
        edge.set_vertex(1, pose_vertices[pose_idx])
        edge.set_measurement(observation)
        edge.set_information(np.eye(2))
        kernel = g2o.RobustKernelHuber(np.sqrt(5.991))
        edge.set_robust_kernel(kernel)
        edge.set_parameter_id(0, 0)
        optimizer.add_edge(edge)

    # Optimize the graph
    optimizer.initialize_optimization()
    optimizer.optimize(10)

    # Extract the optimized poses and points
    optimized_poses = [
        pose_vertex.estimate().to_homogeneous_matrix() for pose_vertex in pose_vertices
    ]
    optimized_points_3D = [point_vertex.estimate() for point_vertex in point_vertices]

    return optimized_poses, optimized_points_3D


def local_bundle_adjustment(poses, points_3D, observations):
    return optimize_bundle_adjustment(poses, points_3D, observations, local=True)


def global_bundle_adjustment(poses, points_3D, observations):
    return optimize_bundle_adjustment(poses, points_3D, observations, local=False)


# vSLAM pipeline
def vslam_pipeline(image_stream, intrinsic_matrix):
    detector = cv2.ORB_create()
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    local_map = []
    global_map = []

    prev_keypoints = None
    prev_descriptors = None
    prev_pose = np.eye(4)

    poses = []
    points_3D = []
    observations = []
    keyframe_buffer = []

    for img in image_stream:
        keypoints, descriptors = extract_features(img, detector)

        if prev_keypoints is not None:
            matches = match_features(prev_descriptors, descriptors, matcher)
            R, t = estimate_motion(matches, prev_keypoints, keypoints, intrinsic_matrix)

            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = t.reshape(-1)
            current_pose = prev_pose @ T

            new_points_3D = triangulate_points(
                matches, prev_keypoints, keypoints, R, t, intrinsic_matrix
            )
            points_3D.extend(new_points_3D)

            for match, point_3D_idx in zip(
                matches, range(len(points_3D) - len(new_points_3D), len(points_3D))
            ):
                observations.append(
                    (point_3D_idx, len(poses) - 1, prev_keypoints[match.queryIdx].pt)
                )
                observations.append(
                    (point_3D_idx, len(poses), keypoints[match.trainIdx].pt)
                )

            optimized_poses, optimized_points_3D = local_bundle_adjustment(
                poses + [current_pose], points_3D, observations
            )

            # Update the local map
            local_map = optimized_points_3D

            # Detect loop closure
            loop_closure_idx = detect_loop_closure(
                keypoints, descriptors, keyframe_buffer, matcher
            )

            if loop_closure_idx is not None:
                # Perform global optimization
                optimized_poses, optimized_points_3D = global_bundle_adjustment(
                    poses, points_3D, observations
                )

                # Update the global map
                global_map = optimized_points_3D

                # Optionally, you can update the local map with the optimized poses and points
                local_map = optimized_points_3D

            # Add current frame as a keyframe
            poses.append(current_pose)
            keyframe_buffer.append((keypoints, descriptors))

            # Update the previous pose
            prev_pose = current_pose

        prev_keypoints = keypoints
        prev_descriptors = descriptors

    return poses, points_3D


# Include all the functions (extract_features, match_features, estimate_motion, triangulate_points,
# local_bundle_adjustment, detect_loop_closure, global_bundle_adjustment, and vslam_pipeline) here


def read_images_from_folder(folder_path, image_format="jpg"):
    filenames = sorted(os.listdir(folder_path))
    image_files = [f for f in filenames if f.endswith(image_format)]
    images = []

    for image_file in image_files:
        img = cv2.imread(os.path.join(folder_path, image_file), cv2.IMREAD_GRAYSCALE)
        images.append(img)

    return images


def visualize_map(poses, points_3D):
    fig = plt.figure()
    ax = fig.add_subplot(projection="3d")

    # Plot the trajectory
    x = [pose[0, 3] for pose in poses]
    y = [pose[1, 3] for pose in poses]
    z = [pose[2, 3] for pose in poses]
    ax.plot(x, y, z, "b-", label="Trajectory")

    # Plot the 3D points
    x = [point_3D[0] for point_3D in points_3D]
    y = [point_3D[1] for point_3D in points_3D]
    z = [point_3D[2] for point_3D in points_3D]
    ax.scatter(x, y, z, c="r", marker="o", s=1, label="Map points")

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.legend()

    plt.show()


# Set the path to the image folder
folder_path = "test_images"

# Set the intrinsic matrix
intrinsic_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])

# Read images from the folder
images = read_images_from_folder(folder_path)

# Process images using the vSLAM pipeline
# Process images using the vSLAM pipeline
poses, points_3D = vslam_pipeline(images, intrinsic_matrix)

# Visualize the map
visualize_map(poses, points_3D)
