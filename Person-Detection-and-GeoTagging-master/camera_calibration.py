#!/usr/bin/env python3
"""
Camera Calibration Tool
Uses existing calibration images from calibration_images/
Supports numeric filenames: 00001.jpg → 00252.jpg
"""

import cv2
import numpy as np
import glob
import os
import yaml

# =====================================================
# CONFIGURATION
# =====================================================

CHECKERBOARD_SIZE = (6, 9)        # Internal corners (cols, rows)
SQUARE_SIZE = 0.025               # meters (25mm)

CALIBRATION_DIR = "calibration_images"
OUTPUT_FILE = "camera_calibration.yaml"
CONFIG_FILE = "config.yaml"

# =====================================================
# HELPERS
# =====================================================

def load_calibration_images():
    """
    Load .jpg images sorted numerically (00001.jpg → 00252.jpg)
    """
    images = sorted(
        glob.glob(os.path.join(CALIBRATION_DIR, "*.jpg")),
        key=lambda x: int(os.path.splitext(os.path.basename(x))[0])
    )

    if len(images) == 0:
        raise RuntimeError(f"No images found in {CALIBRATION_DIR}")

    print(f"✓ Found {len(images)} calibration images")
    print(f"  First: {os.path.basename(images[0])}")
    print(f"  Last : {os.path.basename(images[-1])}")

    return images


# =====================================================
# CALIBRATION
# =====================================================

def calibrate_camera():
    print("\n" + "=" * 60)
    print("CAMERA CALIBRATION - PROCESSING")
    print("=" * 60)

    images = load_calibration_images()

    # Prepare object points (0,0,0) ... (8,5,0)
    objp = np.zeros((CHECKERBOARD_SIZE[0] * CHECKERBOARD_SIZE[1], 3), np.float32)
    objp[:, :2] = np.mgrid[
        0:CHECKERBOARD_SIZE[0],
        0:CHECKERBOARD_SIZE[1]
    ].T.reshape(-1, 2)
    objp *= SQUARE_SIZE

    objpoints = []
    imgpoints = []

    img_size = None
    success = 0

    for fname in images:
        img = cv2.imread(fname)
        if img is None:
            print(f"  ✗ Failed to read {fname}")
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        if img_size is None:
            img_size = gray.shape[::-1]

        found, corners = cv2.findChessboardCorners(
            gray,
            CHECKERBOARD_SIZE,
            cv2.CALIB_CB_ADAPTIVE_THRESH +
            cv2.CALIB_CB_NORMALIZE_IMAGE +
            cv2.CALIB_CB_FAST_CHECK
        )

        if not found:
            print(f"  ✗ {os.path.basename(fname)} - corners not found")
            continue

        # Refine corners
        corners = cv2.cornerSubPix(
            gray,
            corners,
            (11, 11),
            (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        )

        objpoints.append(objp)
        imgpoints.append(corners)
        success += 1

        print(f"  ✓ {os.path.basename(fname)}")

    if success < 10:
        print(f"\n✗ Only {success} valid images found (need ≥ 10)")
        return None

    print(f"\n✓ Running calibration using {success} images")

    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, img_size, None, None
    )

    if not ret:
        print("✗ Calibration failed")
        return None

    # Reprojection error
    total_error = 0
    for i in range(len(objpoints)):
        projected, _ = cv2.projectPoints(
            objpoints[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs
        )
        error = cv2.norm(imgpoints[i], projected, cv2.NORM_L2) / len(projected)
        total_error += error

    mean_error = total_error / len(objpoints)

    fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]
    hfov = 2 * np.degrees(np.arctan(img_size[0] / (2 * fx)))
    vfov = 2 * np.degrees(np.arctan(img_size[1] / (2 * fy)))

    print("\n" + "=" * 60)
    print("CALIBRATION RESULTS")
    print("=" * 60)
    print(f"Reprojection Error: {mean_error:.4f} px")
    print(f"Image Size: {img_size}")
    print("\nCamera Matrix:\n", camera_matrix)
    print("\nDistortion Coeffs:\n", dist_coeffs)
    print(f"\nHFOV: {hfov:.2f}°")
    print(f"VFOV: {vfov:.2f}°")

    calib_data = {
        "image_width": img_size[0],
        "image_height": img_size[1],
        "camera_matrix": camera_matrix.tolist(),
        "distortion_coefficients": dist_coeffs.tolist(),
        "hfov_deg": float(hfov),
        "vfov_deg": float(vfov),
        "reprojection_error": float(mean_error),
        "num_images": success,
    }

    with open(OUTPUT_FILE, "w") as f:
        yaml.dump(calib_data, f)

    print(f"\n✓ Saved calibration to {OUTPUT_FILE}")
    return calib_data


# =====================================================
# CONFIG UPDATE
# =====================================================

def update_config_with_calibration():
    print("\n" + "=" * 60)
    print("UPDATING CONFIG FILE")
    print("=" * 60)

    if not os.path.exists(OUTPUT_FILE):
        print("✗ Calibration file not found")
        return

    with open(OUTPUT_FILE) as f:
        calib = yaml.safe_load(f)

    with open(CONFIG_FILE) as f:
        config = yaml.safe_load(f)

    config["camera"]["hfov_deg"] = calib["hfov_deg"]
    config["camera"]["vfov_deg"] = calib["vfov_deg"]
    config["camera"]["camera_matrix"] = calib["camera_matrix"]
    config["camera"]["distortion_coeffs"] = calib["distortion_coefficients"]

    with open(CONFIG_FILE + ".backup", "w") as f:
        yaml.dump(config, f)

    with open(CONFIG_FILE, "w") as f:
        yaml.dump(config, f)

    print("✓ config.yaml updated")
    print("✓ Backup saved as config.yaml.backup")


# =====================================================
# MAIN
# =====================================================

def main():
    import argparse

    parser = argparse.ArgumentParser("Camera Calibration Tool")
    parser.add_argument("--calibrate", action="store_true")
    parser.add_argument("--update-config", action="store_true")
    args = parser.parse_args()

    if args.calibrate:
        calibrate_camera()
    elif args.update_config:
        update_config_with_calibration()
    else:
        print("\nUsage:")
        print("  python camera_calibration.py --calibrate")
        print("  python camera_calibration.py --update-config")


if __name__ == "__main__":
    main()
