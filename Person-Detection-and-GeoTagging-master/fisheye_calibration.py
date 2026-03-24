import cv2
import numpy as np
import glob
import math
import yaml
import os

# ==========================
# USER SETTINGS - HARD-CODED (DO NOT AUTO-DETECT)
# ==========================
IMAGE_GLOB = "./calibration_images/*.jpg"  # folder with your images
BOARD_COLS = 9   # INNER corners horizontally (HARD-CODED)
BOARD_ROWS = 6   # INNER corners vertically (HARD-CODED)
SQUARE_SIZE = 1.0  # arbitrary (does NOT affect FOV)
IMAGE_SIZE = (1920, 1080)

# Pinhole calibration file to initialize fisheye
PINHOLE_CALIB_FILE = "camera_calibration.yaml"
# ==========================

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)

objpoints = []
imgpoints = []

images = sorted(glob.glob(IMAGE_GLOB))
print(f"Found {len(images)} calibration images")
assert len(images) > 5, "Need more calibration images"

def find_chessboard_corners_fisheye(gray, board_size, try_all_flags=False):
    """
    Try multiple detection strategies for fisheye images
    """
    flags_combinations = [
        # Standard flags
        cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE,
        # Without FAST_CHECK (more thorough)
        cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE,
        # With FILTER_QUADS (helps with distorted images)
        cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FILTER_QUADS,
        # All flags
        cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FILTER_QUADS + cv2.CALIB_CB_FAST_CHECK,
    ]
    
    if try_all_flags:
        for flags in flags_combinations:
            ret, corners = cv2.findChessboardCorners(gray, board_size, flags)
            if ret:
                return ret, corners
    else:
        flags = flags_combinations[0]
        ret, corners = cv2.findChessboardCorners(gray, board_size, flags)
        if ret:
            return ret, corners
    
    return False, None

# Load pinhole calibration for initialization
pinhole_fx = None
pinhole_fy = None
pinhole_cx = None
pinhole_cy = None

if os.path.exists(PINHOLE_CALIB_FILE):
    print(f"Loading pinhole calibration from {PINHOLE_CALIB_FILE}...")
    with open(PINHOLE_CALIB_FILE, 'r') as f:
        pinhole_data = yaml.safe_load(f)
        if 'camera_matrix' in pinhole_data:
            K_pinhole = np.array(pinhole_data['camera_matrix'])
            pinhole_fx = K_pinhole[0, 0]
            pinhole_fy = K_pinhole[1, 1]
            pinhole_cx = K_pinhole[0, 2]
            pinhole_cy = K_pinhole[1, 2]
            print(f"  ✓ Loaded: fx={pinhole_fx:.2f}, fy={pinhole_fy:.2f}")
            print(f"  ✓ Reprojection error: {pinhole_data.get('reprojection_error', 'N/A'):.3f} px")
        if 'image_width' in pinhole_data and 'image_height' in pinhole_data:
            IMAGE_SIZE = (pinhole_data['image_width'], pinhole_data['image_height'])
            print(f"  ✓ Image size: {IMAGE_SIZE}")
else:
    print(f"⚠ Pinhole calibration file not found: {PINHOLE_CALIB_FILE}")
    print("  Will use default initialization")

# Create objp with HARD-CODED size (8x5 inner corners = 40 points)
objp = np.zeros((BOARD_ROWS * BOARD_COLS, 3), np.float64)
objp[:, :2] = np.mgrid[0:BOARD_COLS, 0:BOARD_ROWS].T.reshape(-1, 2)
objp *= SQUARE_SIZE

print(f"\nUsing HARD-CODED board size: {BOARD_COLS}x{BOARD_ROWS} inner corners ({BOARD_ROWS * BOARD_COLS} points)")
print("Processing images...\n")

successful_images = 0

failed_count = 0
for fname in images:
    img = cv2.imread(fname)
    if img is None:
        print(f"Warning: Could not read image {fname}")
        continue
    
    # Update IMAGE_SIZE from first image
    if successful_images == 0:
        IMAGE_SIZE = (img.shape[1], img.shape[0])
        print(f"Image size: {IMAGE_SIZE}")
    
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Try detection with multiple flag combinations (using HARD-CODED size)
    ret, corners = find_chessboard_corners_fisheye(
        gray, 
        (BOARD_COLS, BOARD_ROWS), 
        try_all_flags=True
    )

    if ret:
        corners = cv2.cornerSubPix(
            gray, corners, (11,11), (-1,-1), criteria
        )
        objpoints.append(objp)
        imgpoints.append(corners)
        successful_images += 1
        if successful_images <= 5 or successful_images % 10 == 0:
            print(f"✓ Found corners in: {fname} ({successful_images} images)")
        else:
            print(".", end="", flush=True)
    else:
        print(f"✗ No corners found in: {fname}")
        failed_count += 1

print(f"\nSuccessfully processed {successful_images} out of {len(images)} images")

# Check if we have enough valid images
if len(objpoints) == 0:
    raise ValueError(
        f"No chessboard corners found in any images!\n"
        f"Please check:\n"
        f"  1. Chessboard size is correct (inner corners: {BOARD_COLS}x{BOARD_ROWS})\n"
        f"  2. Images contain a clear chessboard pattern\n"
        f"  3. Chessboard is visible and not too distorted\n"
        f"  4. Image path is correct: {IMAGE_GLOB}"
    )

if len(objpoints) < 3:
    raise ValueError(
        f"Need at least 3 images with detected corners, but only found {len(objpoints)}.\n"
        f"Please add more calibration images with visible chessboard patterns."
    )

# reshape for fisheye (must be Nx1x3 for objpoints, Nx1x2 for imgpoints)
objpoints_fisheye = []
imgpoints_fisheye = []

for op, ip in zip(objpoints, imgpoints):
    # Convert to numpy arrays and ensure correct shape and type
    op_arr = np.asarray(op, dtype=np.float64)
    ip_arr = np.asarray(ip, dtype=np.float64)
    
    # Reshape objpoints: (N, 3) -> (N, 1, 3)
    # objp is created as (BOARD_ROWS * BOARD_COLS, 3)
    if op_arr.ndim == 2 and op_arr.shape[1] == 3:
        op_reshaped = op_arr.reshape(-1, 1, 3)
    else:
        op_reshaped = op_arr.reshape(-1, 1, 3)
    
    # Reshape imgpoints: corners from findChessboardCorners is (N, 1, 2)
    if ip_arr.ndim == 3 and ip_arr.shape[1] == 1 and ip_arr.shape[2] == 2:
        # Already in correct shape (N, 1, 2)
        ip_reshaped = ip_arr
    elif ip_arr.ndim == 2 and ip_arr.shape[1] == 2:
        # Shape is (N, 2), need to add dimension
        ip_reshaped = ip_arr.reshape(-1, 1, 2)
    else:
        # Force reshape
        ip_reshaped = ip_arr.reshape(-1, 1, 2)
    
    objpoints_fisheye.append(op_reshaped)
    imgpoints_fisheye.append(ip_reshaped)

print(f"\nUsing {len(objpoints_fisheye)} images for calibration with image size {IMAGE_SIZE}")

# Initialize camera matrix using pinhole calibration results
# This is critical for accurate fisheye calibration
K = np.zeros((3, 3), dtype=np.float64)

if pinhole_fx is not None and pinhole_fy is not None:
    # Use pinhole calibration to initialize fisheye
    K[0, 0] = pinhole_fx  # fx from pinhole
    K[1, 1] = pinhole_fy  # fy from pinhole
    K[0, 2] = pinhole_cx if pinhole_cx is not None else IMAGE_SIZE[0] / 2.0  # cx
    K[1, 2] = pinhole_cy if pinhole_cy is not None else IMAGE_SIZE[1] / 2.0  # cy
    print(f"\n✓ Initializing fisheye with pinhole results: fx={pinhole_fx:.2f}, fy={pinhole_fy:.2f}")
else:
    # Fallback: use image dimensions (less accurate)
    K[0, 0] = IMAGE_SIZE[0]  # fx
    K[1, 1] = IMAGE_SIZE[1]  # fy
    K[0, 2] = IMAGE_SIZE[0] / 2.0  # cx
    K[1, 2] = IMAGE_SIZE[1] / 2.0  # cy
    print(f"\n⚠ Using default initialization (pinhole calibration not found)")

K[2, 2] = 1.0

# Initialize distortion coefficients (fisheye uses 4 coefficients)
D = np.zeros((4, 1), dtype=np.float64)

# Validate data before calibration
print(f"Validating calibration data...")
print(f"  Number of images: {len(objpoints_fisheye)}")
print(f"  Points per image: {objpoints_fisheye[0].shape[0]}")
print(f"  Object points shape: {objpoints_fisheye[0].shape}")
print(f"  Image points shape: {imgpoints_fisheye[0].shape}")

# fisheye calibration
print("\nRunning fisheye calibration...")
print("This may take a moment...")

# Save original K initialization (from pinhole calibration)
K_original = K.copy()
D_original = D.copy()

# Try calibration with different flag combinations
calibration_success = False
flag_combinations = [
    (cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC + cv2.fisheye.CALIB_USE_INTRINSIC_GUESS, "With intrinsic guess (recommended)"),
    (cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC, "Standard flags"),
    (cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC + cv2.fisheye.CALIB_CHECK_COND, "With condition check"),
    (0, "No flags (default)"),
]

for flags, desc in flag_combinations:
    try:
        print(f"  Trying: {desc}...")
        # Use original K and D for each attempt (from pinhole calibration)
        K_attempt = K_original.copy()
        D_attempt = D_original.copy()
        
        rms, K_result, D_result, rvecs, tvecs = cv2.fisheye.calibrate(
            objpoints_fisheye,
            imgpoints_fisheye,
            IMAGE_SIZE,
            K_attempt,
            D_attempt,
            flags=flags,
            criteria=criteria
        )
        K = K_result
        D = D_result
        calibration_success = True
        print(f"  ✓ Success with {desc}")
        break
    except cv2.error as e:
        error_msg = str(e)
        print(f"  ✗ Failed: {error_msg[:150]}")
        continue

if not calibration_success:
    raise RuntimeError(
        "Fisheye calibration failed with all flag combinations.\n"
        "This might indicate:\n"
        "  1. Insufficient pose diversity in calibration images\n"
        "  2. Chessboard too close to edges or too distorted\n"
        "  3. Need more calibration images with different angles/distances"
    )

print("\n=== FISHEYE CALIBRATION RESULTS ===")
print("RMS reprojection error:", rms)
print("\nCamera matrix K:\n", K)
print("\nDistortion coefficients D:\n", D)

fx = K[0,0]
fy = K[1,1]

# ==========================
# TRUE ANGULAR FOV COMPUTATION
# ==========================

def fisheye_fov(f, image_half_size):
    """
    Computes angular FOV using fisheye geometry
    """
    theta = math.atan(image_half_size / f)
    return 2 * math.degrees(theta)

HFOV = fisheye_fov(fx, IMAGE_SIZE[0] / 2)
VFOV = fisheye_fov(fy, IMAGE_SIZE[1] / 2)

print("\n=== TRUE ANGULAR FOV ===")
print(f"HFOV = {HFOV:.3f} degrees")
print(f"VFOV = {VFOV:.3f} degrees")

# diagonal FOV
diag = math.hypot(IMAGE_SIZE[0], IMAGE_SIZE[1]) / 2
f_diag = (fx + fy) / 2
DFOV = 2 * math.degrees(math.atan(diag / f_diag))

print(f"Diagonal FOV = {DFOV:.3f} degrees")
