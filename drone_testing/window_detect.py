import cv2
import numpy as np
import pyzed.sl as sl

HSV_RANGES = {
    "blue": [(np.array([95, 80, 40]), np.array([130, 255, 255]))],
    "red": [
        (np.array([0, 100, 60]), np.array([10, 255, 255])),
        (np.array([170, 100, 60]), np.array([180, 255, 255])),
    ],
    "green": [(np.array([35, 40, 30]), np.array([90, 255, 255]))],
}


def get_median_depth(depth_img, u, v, box=3):
    h, w = depth_img.shape[:2]
    u = min(max(u, box), w - 1 - box)
    v = min(max(v, box), h - 1 - box)
    depth_values = depth_img[v - box:v + box + 1, u - box:u + box + 1].flatten()
    valid_depths = depth_values[(depth_values > 0) & (~np.isnan(depth_values)) & (~np.isinf(depth_values))]
    if len(valid_depths) > 0:
        return np.median(valid_depths)
    return 0.0


def sample_corner_depth(depth_img, u, v, center, inset=10, box=4):
    cu, cv = center
    du, dv = cu - u, cv - v
    norm = np.hypot(du, dv) + 1e-6
    su = int(round(u + inset * du / norm))
    sv = int(round(v + inset * dv / norm))
    su = min(max(su, 0), depth_img.shape[1] - 1)
    sv = min(max(sv, 0), depth_img.shape[0] - 1)
    return get_median_depth(depth_img, su, sv, box), (su, sv)


def hsv_mask(hsv_image, color):
    mask = None
    for lo, hi in HSV_RANGES[color]:
        m = cv2.inRange(hsv_image, lo, hi)
        mask = m if mask is None else cv2.bitwise_or(mask, m)

    # close_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
    # mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel, iterations=2)

    open_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (4, 4))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel, iterations=2)

    # eroding_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (6, 6))
    # mask = cv2.erode(mask,eroding_kernel, iterations=1)

    dialate_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (4, 4))
    mask = cv2.dilate(mask,dialate_kernel, iterations=2)

    return mask


def approx_quad(contour):
    peri = cv2.arcLength(contour, True)
    for eps in np.linspace(0.01, 0.06, 6):
        approx = cv2.approxPolyDP(contour, eps * peri, True)
        if len(approx) == 4:
            return approx
    return None


def window_detection(mask, min_area=1500):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < min_area:
        return None
    hull = cv2.convexHull(largest)
    approx = approx_quad(hull)
    if approx is None or not cv2.isContourConvex(approx):
        return None
    return approx


def filter_depth(new_d, prev_d, alpha=0.3):
    if new_d is not None and new_d > 0:
        if prev_d is None or prev_d == 0:
            return new_d, new_d
        filtered = alpha * new_d + (1 - alpha) * prev_d
        return filtered, filtered
    return prev_d, prev_d


def main(port=30000):
    zed = sl.Camera()
    
    # Configure parameters
    init_params = sl.InitParameters()
    init_params.depth_mode = sl.DEPTH_MODE.NEURAL
    init_params.coordinate_units = sl.UNIT.CENTIMETER
    init_params.set_from_stream("127.0.0.1", port)

    init_params.camera_fps = 15

    print(f"Connecting to port {port}...")
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print(f"Error connecting to stream: {err}")
        return

    print("Successfully connected! Press 'q' to stop.")

    cam_info = zed.get_camera_information()
    calib = cam_info.camera_configuration.calibration_parameters.left_cam

    camera_matrix = np.array([
        [calib.fx, 0, calib.cx],
        [0, calib.fy, calib.cy],
        [0, 0, 1]
    ], dtype=np.float32)

    image_zed = sl.Mat()
    depth_zed = sl.Mat()
    runtime_params = sl.RuntimeParameters()

    # State variables
    padding = 5
    smoothed_corners = None
    corner_alpha = 0.4
    depth_alpha = 0.3
    prev_e1 = prev_e2 = prev_e3 = prev_e4 = 0

    while True:
        if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
            
            # Retrieve RGB & DEPTH (Automatically aligned and rectified by SDK)
            zed.retrieve_image(image_zed, sl.VIEW.LEFT)
            zed.retrieve_measure(depth_zed, sl.MEASURE.DEPTH)
            
            cv_rgb_image = image_zed.get_data()
            cv_image = cv2.cvtColor(cv_rgb_image, cv2.COLOR_BGRA2BGR)
            depth_image = depth_zed.get_data()

            hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            green_mask = hsv_mask(hsv_img, "green")
            window_contour = window_detection(green_mask)

            if window_contour is not None:
                window_contour = window_contour.reshape(-1, 2)
                s = np.sum(window_contour, axis=1)
                u1, v1 = window_contour[np.argmin(s)]  
                u3, v3 = window_contour[np.argmax(s)]  

                u1, v1 = max(u1 + padding, 0), max(v1 + padding, 0)
                u3, v3 = min(u3 - padding, cv_image.shape[1] - 1), min(v3 - padding, cv_image.shape[0] - 1)
                

                diff = np.diff(window_contour, axis=1)
                u2, v2 = window_contour[np.argmin(diff)]  
                u4, v4 = window_contour[np.argmax(diff)]  

                u2, v2 = max(u2 - padding, 0), max(v2 + padding, 0)
                u4, v4 = min(u4 + padding, cv_image.shape[1] - 1), min(v4 - padding, cv_image.shape[0] - 1)

                corners = np.array([[u1, v1], [u2, v2], [u3, v3], [u4, v4]], dtype=np.float32)

                if smoothed_corners is None:
                    smoothed_corners = corners
                else:
                    smoothed_corners = (corner_alpha * corners + (1 - corner_alpha) * smoothed_corners)
                
                u1, v1 = smoothed_corners[0].astype(int)
                u2, v2 = smoothed_corners[1].astype(int)
                u3, v3 = smoothed_corners[2].astype(int)
                u4, v4 = smoothed_corners[3].astype(int)

                center = smoothed_corners.mean(axis=0)

                if depth_image is not None:
                    d1, s1 = sample_corner_depth(depth_image, u1, v1, center)
                    d2, s2 = sample_corner_depth(depth_image, u2, v2, center)
                    d3, s3 = sample_corner_depth(depth_image, u3, v3, center)
                    d4, s4 = sample_corner_depth(depth_image, u4, v4, center)

                    for sp in (s1, s2, s3, s4):
                        cv2.circle(cv_image, sp, 4, (0, 0, 255), -1)

                    d1, prev_e1 = filter_depth(d1, prev_e1, depth_alpha)
                    d2, prev_e2 = filter_depth(d2, prev_e2, depth_alpha)
                    d3, prev_e3 = filter_depth(d3, prev_e3, depth_alpha)
                    d4, prev_e4 = filter_depth(d4, prev_e4, depth_alpha)
                
                cv2.drawContours(cv_image, [window_contour], -1, (255, 0, 255), 3)
                cv2.putText(cv_image, "Window Detected", (u1 - 10, v1 - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                
                for pt, label, d in zip([(u1, v1), (u2, v2), (u3, v3), (u4, v4)], ['d1', 'd2', 'd3', 'd4'], [d1, d2, d3, d4]):
                    cv2.circle(cv_image, pt, 6, (255, 0, 0), -1) 
                    cv2.putText(cv_image, f"{label}={d:.2f}", (pt[0] + 10, pt[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)


            cv2.imshow("ZED Image Processing", cv_image)
            cv2.imshow("Green Mask", green_mask)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    zed.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main(port=30000)