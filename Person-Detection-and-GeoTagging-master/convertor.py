import os
import shutil
from pathlib import Path
from PIL import Image
from tqdm import tqdm

"""
VisDrone to YOLO Format Converter
Converts VisDrone dataset annotations to YOLO format and filters for pedestrian/people classes only.

VisDrone Format: <bbox_left>,<bbox_top>,<bbox_width>,<bbox_height>,<score>,<object_category>,<truncation>,<occlusion>
YOLO Format: <class_id> <x_center> <y_center> <width> <height> (all normalized 0-1)

VisDrone Classes:
0: ignored regions
1: pedestrian
2: people
3: bicycle
4: car
5: van
6: truck
7: tricycle
8: awning-tricycle
9: bus
10: motor
"""

# Configuration
VISDRONE_ROOT = "Visdrone/VisDrone2019-DET-train"  # Change this to your VisDrone root folder
OUTPUT_ROOT = "Visdrone/Output"  # Output folder for YOLO format dataset

# Classes to keep (1=pedestrian, 2=people)
KEEP_CLASSES = [1, 2]
CLASS_MAPPING = {1: 0, 2: 1}  # Map to YOLO class IDs: pedestrian=0, people=1
CLASS_NAMES = ['pedestrian', 'people']


def convert_visdrone_to_yolo(visdrone_annotation, img_width, img_height, keep_classes=KEEP_CLASSES):
    """
    Convert VisDrone annotation line to YOLO format.
    
    Args:
        visdrone_annotation: Single line from VisDrone annotation file
        img_width: Image width in pixels
        img_height: Image height in pixels
        keep_classes: List of VisDrone class IDs to keep
    
    Returns:
        YOLO formatted string or None if class should be filtered out
    """
    parts = visdrone_annotation.strip().split(',')
    
    if len(parts) < 8:
        return None
    
    bbox_left = int(parts[0])
    bbox_top = int(parts[1])
    bbox_width = int(parts[2])
    bbox_height = int(parts[3])
    score = int(parts[4])
    object_category = int(parts[5])
    truncation = int(parts[6])
    occlusion = int(parts[7])
    
    # Filter out ignored regions and unwanted classes
    if object_category not in keep_classes:
        return None
    
    # Filter out invalid bounding boxes
    if bbox_width <= 0 or bbox_height <= 0:
        return None
    
    # Optional: Filter heavily occluded or truncated objects
    # if occlusion == 2 or truncation == 2:  # Uncomment to filter heavily occluded/truncated
    #     return None
    
    # Convert to YOLO format (normalized coordinates)
    x_center = (bbox_left + bbox_width / 2) / img_width
    y_center = (bbox_top + bbox_height / 2) / img_height
    width = bbox_width / img_width
    height = bbox_height / img_height
    
    # Clip values to [0, 1] range
    x_center = max(0, min(1, x_center))
    y_center = max(0, min(1, y_center))
    width = max(0, min(1, width))
    height = max(0, min(1, height))
    
    # Map to YOLO class ID
    yolo_class_id = CLASS_MAPPING[object_category]
    
    return f"{yolo_class_id} {x_center:.6f} {y_center:.6f} {width:.6f} {height:.6f}"


def process_dataset(visdrone_root, output_root, split='train'):
    """
    Process VisDrone dataset and convert to YOLO format.
    
    Args:
        visdrone_root: Path to VisDrone dataset root
        output_root: Path to output YOLO format dataset
        split: Dataset split ('train' or 'val')
    """
    print(f"\n{'='*60}")
    print(f"Processing {split} split...")
    print(f"{'='*60}\n")
    
    # Define paths
    visdrone_images_dir = os.path.join(visdrone_root, 'images')
    visdrone_annotations_dir = os.path.join(visdrone_root, 'annotations')
    
    output_images_dir = os.path.join(output_root, 'images', split)
    output_labels_dir = os.path.join(output_root, 'labels', split)
    
    # Create output directories
    os.makedirs(output_images_dir, exist_ok=True)
    os.makedirs(output_labels_dir, exist_ok=True)
    
    # Get list of images
    image_files = [f for f in os.listdir(visdrone_images_dir) if f.endswith(('.jpg', '.png', '.jpeg'))]
    
    print(f"Found {len(image_files)} images in {split} split")
    
    converted_count = 0
    skipped_count = 0
    total_annotations = 0
    
    # Process each image
    for img_file in tqdm(image_files, desc=f"Converting {split}"):
        img_path = os.path.join(visdrone_images_dir, img_file)
        annotation_file = os.path.splitext(img_file)[0] + '.txt'
        annotation_path = os.path.join(visdrone_annotations_dir, annotation_file)
        
        # Check if annotation file exists
        if not os.path.exists(annotation_path):
            skipped_count += 1
            continue
        
        # Get image dimensions
        try:
            with Image.open(img_path) as img:
                img_width, img_height = img.size
        except Exception as e:
            print(f"Error reading image {img_file}: {e}")
            skipped_count += 1
            continue
        
        # Read VisDrone annotations
        with open(annotation_path, 'r') as f:
            visdrone_annotations = f.readlines()
        
        # Convert annotations to YOLO format
        yolo_annotations = []
        for annotation in visdrone_annotations:
            yolo_annotation = convert_visdrone_to_yolo(annotation, img_width, img_height)
            if yolo_annotation:
                yolo_annotations.append(yolo_annotation)
        
        # Skip images with no valid annotations
        if len(yolo_annotations) == 0:
            skipped_count += 1
            continue
        
        # Copy image to output directory
        output_img_path = os.path.join(output_images_dir, img_file)
        shutil.copy2(img_path, output_img_path)
        
        # Save YOLO format annotations
        output_label_path = os.path.join(output_labels_dir, annotation_file)
        with open(output_label_path, 'w') as f:
            f.write('\n'.join(yolo_annotations))
        
        converted_count += 1
        total_annotations += len(yolo_annotations)
    
    print(f"\n{split.capitalize()} split conversion complete:")
    print(f"  ✓ Converted: {converted_count} images")
    print(f"  ✓ Total annotations: {total_annotations}")
    print(f"  ✓ Avg annotations per image: {total_annotations/converted_count:.2f}")
    print(f"  ✗ Skipped: {skipped_count} images (no valid annotations)")


def create_yaml_file(output_root):
    """Create YAML configuration file for YOLO training."""
    yaml_content = f"""# VisDrone Dataset - Pedestrian and People Detection
# Converted to YOLO format

path: {os.path.abspath(output_root)}
train: images/train
val: images/val

# Classes
nc: 2
names:
  0: pedestrian
  1: people

# Dataset info
# Source: VisDrone2019-DET
# Filtered for pedestrian and people classes only
"""
    
    yaml_path = os.path.join(output_root, 'visdrone.yaml')
    with open(yaml_path, 'w') as f:
        f.write(yaml_content)
    
    print(f"\n✓ YAML configuration saved to: {yaml_path}")
    return yaml_path


def main():
    """Main conversion pipeline."""
    print("\n" + "="*60)
    print("VisDrone to YOLO Format Converter")
    print("Converting for Pedestrian and People Detection")
    print("="*60)
    
    # Check if input directory exists
    if not os.path.exists(VISDRONE_ROOT):
        print(f"\n✗ Error: VisDrone dataset not found at: {VISDRONE_ROOT}")
        print("Please update VISDRONE_ROOT variable with correct path")
        return
    
    # Process train split
    process_dataset(VISDRONE_ROOT, OUTPUT_ROOT, split='train')
    
    # Process val split if exists
    visdrone_val_root = VISDRONE_ROOT.replace('train', 'val')
    if os.path.exists(visdrone_val_root):
        process_dataset(visdrone_val_root, OUTPUT_ROOT, split='val')
    else:
        print(f"\n⚠ Warning: Validation set not found at {visdrone_val_root}")
        print("If you have a separate validation set, run this script again with updated path")
    
    # Create YAML configuration file
    yaml_path = create_yaml_file(OUTPUT_ROOT)
    
    print("\n" + "="*60)
    print("Conversion Complete!")
    print("="*60)
    print(f"\nYour dataset is ready at: {os.path.abspath(OUTPUT_ROOT)}")
    print(f"\nDataset structure:")
    print(f"  {OUTPUT_ROOT}/")
    print(f"  ├── images/")
    print(f"  │   ├── train/")
    print(f"  │   └── val/")
    print(f"  ├── labels/")
    print(f"  │   ├── train/")
    print(f"  │   └── val/")
    print(f"  └── visdrone.yaml")
    print(f"\nYou can now use this dataset to train YOLOv11!")
    print(f"Update the training script to use: {yaml_path}")


if __name__ == "__main__":
    main()