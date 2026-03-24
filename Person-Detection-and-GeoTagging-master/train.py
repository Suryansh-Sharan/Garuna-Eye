import os
from ultralytics import YOLO
import yaml

# Configuration
BASE_DIR = "/home/suryansh/All-Coding-FIles/YOLO-Trained/Visdrone/Output"  # Your converted dataset
IMAGES_DIR = os.path.join(BASE_DIR, "images")
LABELS_DIR = os.path.join(BASE_DIR, "labels")

# Verify dataset structure
print("Verifying dataset structure...")
train_images = os.path.join(IMAGES_DIR, 'train')
val_images = os.path.join(IMAGES_DIR, 'val')
train_labels = os.path.join(LABELS_DIR, 'train')
val_labels = os.path.join(LABELS_DIR, 'val')

for path, name in [(train_images, 'Train images'), (val_images, 'Val images'),
                    (train_labels, 'Train labels'), (val_labels, 'Val labels')]:
    if os.path.exists(path):
        count = len([f for f in os.listdir(path) if os.path.isfile(os.path.join(path, f))])
        print(f"✓ {name}: {count} files found")
    else:
        print(f"✗ {name}: Directory not found at {path}")
        raise FileNotFoundError(f"Required directory not found: {path}")

# Create YAML configuration file for the dataset
dataset_yaml = {
    'path': os.path.abspath(BASE_DIR),
    'train': os.path.join('images', 'train'),
    'val': os.path.join('images', 'val'),
    'nc': 2,  # Training only 2 classes: pedestrian and people
    'names': [
        'pedestrian',
        'people'
    ]
}

# Save YAML file (using the one from conversion)
yaml_path = os.path.join(BASE_DIR, 'visdrone.yaml')
if not os.path.exists(yaml_path):
    # Create YAML if it doesn't exist
    with open(yaml_path, 'w') as f:
        yaml.dump(dataset_yaml, f, default_flow_style=False)
    print(f"Dataset configuration saved to: {yaml_path}")
else:
    print(f"Using existing dataset configuration: {yaml_path}")

# Load YOLOv11 model
# Options: yolo11n.pt (nano), yolo11s.pt (small), yolo11m.pt (medium), 
#          yolo11l.pt (large), yolo11x.pt (extra large)
model = YOLO('yolo11n.pt')  # Start with nano model, change as needed

# Training parameters
training_args = {
    'data': yaml_path,
    'epochs': 100,
    'imgsz': 640,
    'batch': 8,  # Reduced for 16GB RAM systems
    'device': 0,  # Use GPU 0, or 'cpu' for CPU training
    'workers': 8,
    'patience': 50,  # Early stopping patience
    'save': True,
    'save_period': 10,  # Save checkpoint every 10 epochs
    'cache': 'disk',  # Use disk caching (already done)
    'project': 'runs/train',
    'name': 'yolo11_visdrone_final',  # Changed name to avoid conflict
    'exist_ok': False,  # Create new experiment folder
    'pretrained': True,
    'optimizer': 'auto',  # Options: SGD, Adam, AdamW, NAdam, RAdam, RMSProp, auto
    'verbose': True,
    'seed': 0,
    'deterministic': True,
    'single_cls': False,
    'rect': False,
    'cos_lr': False,
    'close_mosaic': 10,  # Disable mosaic augmentation for last N epochs
    'resume': False,  # CRITICAL: Must be False for new training
    'amp': True,  # Automatic Mixed Precision training
    'fraction': 1.0,  # Dataset fraction to use
    'profile': False,
    'freeze': None,  # Freeze first N layers, or list of layer indices
    'lr0': 0.01,  # Initial learning rate
    'lrf': 0.01,  # Final learning rate (lr0 * lrf)
    'momentum': 0.937,
    'weight_decay': 0.0005,
    'warmup_epochs': 3.0,
    'warmup_momentum': 0.8,
    'warmup_bias_lr': 0.1,
    'box': 7.5,  # Box loss gain
    'cls': 0.5,  # Classification loss gain
    'dfl': 1.5,  # Distribution focal loss gain
    'pose': 12.0,
    'kobj': 1.0,
    'label_smoothing': 0.0,
    'nbs': 64,
    'hsv_h': 0.015,  # HSV-Hue augmentation
    'hsv_s': 0.7,  # HSV-Saturation augmentation
    'hsv_v': 0.4,  # HSV-Value augmentation
    'degrees': 0.0,  # Rotation augmentation
    'translate': 0.1,  # Translation augmentation
    'scale': 0.5,  # Scale augmentation
    'shear': 0.0,  # Shear augmentation
    'perspective': 0.0,  # Perspective augmentation
    'flipud': 0.0,  # Vertical flip probability
    'fliplr': 0.5,  # Horizontal flip probability
    'mosaic': 1.0,  # Mosaic augmentation probability (good for small objects)
    'mixup': 0.0,  # Mixup augmentation probability
    'copy_paste': 0.0,  # Copy-paste augmentation probability
}

# Start training
print("\n" + "="*50)
print("Starting YOLOv11 Training on VisDrone Dataset")
print("="*50 + "\n")

results = model.train(**training_args)

print("\n" + "="*50)
print("Training Complete!")
print("="*50)
print(f"\nBest model saved at: runs/train/yolo11_visdrone/weights/best.pt")
print(f"Last model saved at: runs/train/yolo11_visdrone/weights/last.pt")

# Validate the trained model
print("\n" + "="*50)
print("Validating Model on Validation Set")
print("="*50 + "\n")

metrics = model.val()

print("\nValidation Metrics:")
print(f"mAP50: {metrics.box.map50:.4f}")
print(f"mAP50-95: {metrics.box.map:.4f}")

# Optional: Test prediction on a sample image
# Uncomment the following lines to test
"""
print("\nTesting prediction on sample image...")
sample_image = os.path.join(IMAGES_DIR, 'val', os.listdir(os.path.join(IMAGES_DIR, 'val'))[0])
results = model.predict(source=sample_image, save=True, conf=0.25)
print(f"Prediction saved to: runs/detect/predict")
"""