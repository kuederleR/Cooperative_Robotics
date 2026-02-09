import cv2
import numpy as np
from math import sqrt
import os.path as osp
from pathlib import Path

TRACKING_CAM = "down"

# Replace the intrinsics loader to support multiple key names and fisheye
INTRINSICS_PATH = Path(__file__).with_name("opencv_tracking_"+TRACKING_CAM+"_intrinsics.yml")
def _load_intrinsics(path: Path):
    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        print(f"Warning: cannot open intrinsics at {path.resolve()}; pose estimation disabled.")
        return None, None, False
    def _read_first(keys):
        for k in keys:
            node = fs.getNode(k)
            if not node.empty():
                mat = node.mat()
                if mat is not None:
                    return mat
        return None
    K = _read_first(["camera_matrix", "K", "M"])
    D = _read_first(["distortion_coefficients", "distCoeffs", "D"])
    model_node = fs.getNode("distortion_model")
    model = model_node.string() if not model_node.empty() else ""
    fs.release()

    is_fisheye = model.lower() == "fisheye"
    if K is not None:
        K = K.astype(np.float64)
    if D is not None:
        D = D.astype(np.float64).reshape(-1, 1)
    if K is None or D is None:
        print(f"Warning: intrinsics missing keys (looked for camera_matrix/K/M and distortion_coefficients/distCoeffs/D) in {path.resolve()}; pose estimation disabled.")
    else:
        msg = "fisheye" if is_fisheye else "pinhole"
        print(f"Intrinsics loaded ({msg}): K shape {K.shape}, D len {D.size}")
    return K, D, is_fisheye

camera_matrix, dist_coeffs, is_fisheye = _load_intrinsics(INTRINSICS_PATH)

def main():
    # Initialize camera
    cap = cv2.VideoCapture("http://10.152.70.62/video_raw/tracking_" + TRACKING_CAM)
    
    # Load ArUco dictionary and detector parameters
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_100)
    aruco_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    marker_real_size = 0.112  # Marker size in meters

    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detect ArUco markers
        corners, ids, rejected = detector.detectMarkers(gray)
        
        if ids is not None:
            if camera_matrix is not None and dist_coeffs is not None:
                try:
                    rvecs, tvecs, _ = my_estimatePoseSingleMarkers(
                        corners, marker_real_size, camera_matrix, dist_coeffs, is_fisheye
                    )
                except Exception as e:
                    print(f"Pose estimation failed: {e}")
                    rvecs, tvecs = [], []
            else:
                # No intrinsics; cannot compute pose
                rvecs, tvecs = [], []
            x, y, z = 0, 0, 0

            for i, t in enumerate(tvecs):
                x = t[0][0]
                y = t[1][0]
                z = t[2][0]
                print(f"Marker ID: {ids[i][0]}, Position (tvec): x={x:.2f}, y={y:.2f}, z={z:.2f}")

            dist = sqrt(x**2 + y**2 + z**2)
            
            # Draw detected markers
            cv2.aruco.drawDetectedMarkers(frame, corners)
            
            # Print marker information
            for i, marker_id in enumerate(ids):
                
                # Calculate center of marker
                center = np.mean(corners[i][0], axis=0).astype(int)
                # cv2.circle(frame, tuple(center), 5, (0, 255, 0), -1)
                cv2.putText(frame, f'Dist: {dist:.2f}m', 
                           (center[0] + 10, center[1] + 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        
        # Display the frame
        cv2.imshow('ArUco Tracker', frame)
        
        # Exit on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # Cleanup
    cap.release()
    cv2.destroyAllWindows()

# Source - https://stackoverflow.com/a
# Posted by M lab, modified by community. See post 'Timeline' for change history
# Retrieved 2025-11-12, License - CC BY-SA 4.0

def my_estimatePoseSingleMarkers(corners, marker_size, mtx, distortion, fisheye=False):
    # Fail fast if intrinsics absent
    if mtx is None or distortion is None:
        raise ValueError("Camera intrinsics are None; cannot estimate pose.")
    marker_points = np.array([[-marker_size / 2,  marker_size / 2, 0],
                              [ marker_size / 2,  marker_size / 2, 0],
                              [ marker_size / 2, -marker_size / 2, 0],
                              [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float64)
    trash, rvecs, tvecs = [], [], []
    for c in corners:
        img_pts = c.reshape(-1, 2).astype(np.float64)  # (4,2)
        if fisheye:
            # Undistort to ideal pixel coordinates using P=K, then solve with zero distortion
            pts = img_pts.reshape(-1, 1, 2)
            undist = cv2.fisheye.undistortPoints(pts, mtx, distortion, P=mtx)  # -> (N,1,2) pixels
            img_pts = undist.reshape(-1, 2)
            dist = None
        else:
            dist = distortion
        ret, rvec, tvec = cv2.solvePnP(marker_points, img_pts, mtx, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        rvecs.append(rvec)
        tvecs.append(tvec)
        trash.append(ret)
    return rvecs, tvecs, trash

if __name__ == "__main__":
    main()