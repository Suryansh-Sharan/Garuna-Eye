#!/usr/bin/env python3
"""
Pre-Flight System Check
Validates all components before testing/flying
"""

import os
import sys
import cv2
import yaml
import socket
import json
import time
from pathlib import Path

# Colors for terminal output
class Colors:
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    RESET = '\033[0m'
    BOLD = '\033[1m'

def print_header(text):
    print(f"\n{Colors.BOLD}{Colors.BLUE}{'='*70}{Colors.RESET}")
    print(f"{Colors.BOLD}{text:^70}{Colors.RESET}")
    print(f"{Colors.BOLD}{Colors.BLUE}{'='*70}{Colors.RESET}\n")

def print_check(name, passed, details=""):
    status = f"{Colors.GREEN}✓" if passed else f"{Colors.RED}✗"
    print(f"{status} {name:<40}{Colors.RESET} {details}")
    return passed

def check_python_version():
    """Check Python version"""
    version = sys.version_info
    required = (3, 7)
    passed = version >= required
    details = f"v{version.major}.{version.minor}.{version.micro}"
    return print_check("Python version", passed, details)

def check_dependencies():
    """Check if required packages are installed"""
    print("\nChecking Python packages...")
    
    packages = {
        'cv2': 'opencv-python',
        'numpy': 'numpy',
        'yaml': 'pyyaml',
        'ultralytics': 'ultralytics',
        'folium': 'folium'
    }
    
    all_ok = True
    for module, package in packages.items():
        try:
            __import__(module)
            print_check(f"  {package}", True)
        except ImportError:
            print_check(f"  {package}", False, "MISSING")
            all_ok = False
    
    return all_ok

def check_config_file():
    """Check if config.yaml exists and is valid"""
    print("\nChecking configuration...")
    
    if not os.path.exists("config.yaml"):
        return print_check("config.yaml", False, "File not found")
    
    try:
        with open("config.yaml") as f:
            config = yaml.safe_load(f)
        
        # Check required fields
        required_fields = [
            ('camera', 'hfov_deg'),
            ('camera', 'vfov_deg'),
            ('yolo', 'model_path'),
            ('network', 'udp_port')
        ]
        
        all_ok = True
        for section, field in required_fields:
            if section not in config or field not in config[section]:
                print_check(f"  {section}.{field}", False, "Missing")
                all_ok = False
            else:
                print_check(f"  {section}.{field}", True, 
                           f"{config[section][field]}")
        
        return all_ok
        
    except Exception as e:
        return print_check("config.yaml", False, f"Error: {e}")

def check_yolo_model():
    """Check if YOLO model exists"""
    print("\nChecking YOLO model...")
    
    try:
        with open("config.yaml") as f:
            config = yaml.safe_load(f)
        
        model_path = config['yolo']['model_path']
        
        if not os.path.exists(model_path):
            return print_check("YOLO model file", False, 
                             f"Not found: {model_path}")
        
        # Try loading model
        try:
            from ultralytics import YOLO
            model = YOLO(model_path)
            classes = list(model.names.values())
            print_check("YOLO model loading", True)
            print(f"    Classes: {', '.join(classes[:5])}...")
            return True
        except Exception as e:
            return print_check("YOLO model loading", False, f"Error: {e}")
            
    except Exception as e:
        return print_check("YOLO model", False, f"Config error: {e}")

def check_camera():
    """Check if camera is accessible"""
    print("\nChecking camera...")
    
    try:
        cap = cv2.VideoCapture(0)
        
        if not cap.isOpened():
            return print_check("Camera access", False, "Cannot open camera")
        
        ret, frame = cap.read()
        cap.release()
        
        if not ret:
            return print_check("Camera capture", False, "Cannot capture frame")
        
        h, w = frame.shape[:2]
        print_check("Camera", True, f"{w}x{h}")
        return True
        
    except Exception as e:
        return print_check("Camera", False, f"Error: {e}")

def check_network(laptop_ip=None):
    """Check network connectivity"""
    print("\nChecking network...")
    
    try:
        # Check if we can bind to UDP ports
        with open("config.yaml") as f:
            config = yaml.safe_load(f)
        
        udp_port = config['network']['udp_port']
        
        # Try binding to telemetry port
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind(('0.0.0.0', udp_port))
            sock.close()
            print_check(f"UDP port {udp_port}", True, "Available")
        except:
            print_check(f"UDP port {udp_port}", False, "Port in use or blocked")
            return False
        
        # Try binding to image port
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind(('0.0.0.0', udp_port + 1))
            sock.close()
            print_check(f"UDP port {udp_port + 1}", True, "Available")
        except:
            print_check(f"UDP port {udp_port + 1}", False, "Port in use or blocked")
            return False
        
        # If laptop IP provided, try pinging
        if laptop_ip:
            try:
                # Try connecting (just to test network)
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.settimeout(2)
                sock.connect((laptop_ip, udp_port))
                sock.close()
                print_check(f"Network to {laptop_ip}", True, "Reachable")
            except:
                print_check(f"Network to {laptop_ip}", False, "Cannot reach")
        
        return True
        
    except Exception as e:
        return print_check("Network", False, f"Error: {e}")

def check_directories():
    """Check if output directories exist"""
    print("\nChecking directories...")
    
    dirs = [
        "outputs",
        "outputs/images",
        "outputs/maps"
    ]
    
    all_ok = True
    for directory in dirs:
        if not os.path.exists(directory):
            os.makedirs(directory, exist_ok=True)
            print_check(f"  {directory}", True, "Created")
        else:
            print_check(f"  {directory}", True, "Exists")
    
    return all_ok

def test_detection_speed():
    """Test YOLO inference speed"""
    print("\nTesting detection speed...")
    
    try:
        from ultralytics import YOLO
        import time
        
        with open("config.yaml") as f:
            config = yaml.safe_load(f)
        
        model = YOLO(config['yolo']['model_path'])
        
        # Create dummy image
        import numpy as np
        dummy_img = np.random.randint(0, 255, (720, 1280, 3), dtype=np.uint8)
        
        # Warm up
        _ = model.predict(dummy_img, verbose=False)
        
        # Time it
        times = []
        for _ in range(5):
            start = time.time()
            _ = model.predict(dummy_img, verbose=False)
            times.append(time.time() - start)
        
        avg_time = sum(times) / len(times)
        fps = 1.0 / avg_time
        
        passed = avg_time < 1.0  # Should process faster than 1 sec
        print_check("Detection speed", passed, 
                   f"{avg_time*1000:.0f}ms ({fps:.1f} FPS)")
        
        return passed
        
    except Exception as e:
        return print_check("Detection speed", False, f"Error: {e}")

def check_calibration():
    """Check if camera is calibrated"""
    print("\nChecking camera calibration...")
    
    try:
        with open("config.yaml") as f:
            config = yaml.safe_load(f)
        
        if 'camera_matrix' in config['camera']:
            print_check("Camera calibrated", True, "Using calibration data")
            return True
        else:
            print_check("Camera calibrated", False, 
                       "No calibration - run camera_calibration.py")
            print(f"    {Colors.YELLOW}Not critical, but recommended for accuracy{Colors.RESET}")
            return True  # Not critical
            
    except Exception as e:
        return print_check("Calibration check", False, f"Error: {e}")

def run_full_check(laptop_ip=None):
    """Run all checks"""
    print_header("DRONE GEOTAGGING SYSTEM CHECK")
    
    print(f"{Colors.YELLOW}This script validates your system setup{Colors.RESET}")
    print(f"{Colors.YELLOW}Run this before testing or flying{Colors.RESET}")
    
    checks = []
    
    # Core checks
    print_header("Core System")
    checks.append(("Python", check_python_version()))
    checks.append(("Dependencies", check_dependencies()))
    checks.append(("Config file", check_config_file()))
    checks.append(("Directories", check_directories()))
    
    # Model checks
    print_header("YOLO Model")
    checks.append(("Model", check_yolo_model()))
    checks.append(("Speed test", test_detection_speed()))
    
    # Hardware checks
    print_header("Hardware")
    checks.append(("Camera", check_camera()))
    checks.append(("Calibration", check_calibration()))
    
    # Network checks
    print_header("Network")
    checks.append(("Network", check_network(laptop_ip)))
    
    # Summary
    print_header("SUMMARY")
    
    passed = sum(1 for _, status in checks if status)
    total = len(checks)
    
    print(f"Checks passed: {Colors.GREEN}{passed}{Colors.RESET}/{total}\n")
    
    if passed == total:
        print(f"{Colors.GREEN}{Colors.BOLD}✓ ALL CHECKS PASSED{Colors.RESET}")
        print(f"{Colors.GREEN}System is ready for testing/flying!{Colors.RESET}\n")
        return True
    else:
        print(f"{Colors.RED}{Colors.BOLD}✗ SOME CHECKS FAILED{Colors.RESET}")
        print(f"{Colors.RED}Please fix the issues above before proceeding{Colors.RESET}\n")
        return False

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description="System Check")
    parser.add_argument("--laptop-ip", help="Laptop IP for network test")
    args = parser.parse_args()
    
    success = run_full_check(args.laptop_ip)
    
    if success:
        print("Next steps:")
        print("  1. Run camera calibration: python camera_calibration.py --capture")
        print("  2. Test coordinate math:   python test_geotagging.py")
        print("  3. Indoor hand test:       python manual_test.py")
        print("  4. Live test:              python maingeo.py --config config.yaml --mode live")
    else:
        print("Fix the failed checks above, then run this script again")
    
    sys.exit(0 if success else 1)

if __name__ == "__main__":
    main()