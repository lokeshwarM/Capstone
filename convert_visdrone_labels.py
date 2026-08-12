import os
import glob
from PIL import Image  # pyfly: ignore [missing-import] # type: ignore

def convert_visdrone_to_yolo():
    """
    Converts raw VisDrone annotation files (annotations/*.txt) into standard YOLO format (labels/*.txt).
    VisDrone format: x, y, width, height, score, category, truncation, occlusion
    YOLO format: category_id x_center y_center width height (normalized 0..1)
    """
    print("===================================================================")
    print("  CONVERTING VISDRONE ANNOTATIONS TO YOLO FORMAT")
    print("===================================================================")

    base_dir = os.path.join(os.path.dirname(__file__), "data", "VisDrone", "VisDrone_Full", "VisDrone2019-DET-train")
    annotations_dir = os.path.join(base_dir, "annotations")
    images_dir = os.path.join(base_dir, "images")
    labels_dir = os.path.join(base_dir, "labels")

    os.makedirs(labels_dir, exist_ok=True)

    ann_files = glob.glob(os.path.join(annotations_dir, "*.txt"))
    if not ann_files:
        print(f"[Error] No annotation txt files found in {annotations_dir}")
        return

    print(f"[Converter] Found {len(ann_files)} annotation files. Converting to YOLO format...")

    converted_count = 0
    for ann_path in ann_files:
        filename = os.path.basename(ann_path)
        img_filename = filename.replace(".txt", ".jpg")
        img_path = os.path.join(images_dir, img_filename)

        if not os.path.exists(img_path):
            continue

        try:
            with Image.open(img_path) as img:
                img_w, img_h = img.size
        except Exception:
            continue

        yolo_lines = []
        with open(ann_path, 'r') as f:
            lines = f.readlines()
            for line in lines:
                parts = line.strip().split(',')
                if len(parts) < 6:
                    continue

                x, y, w, h, score, category = [float(p) for p in parts[:6]]
                cat_id = int(category) - 1  # Convert VisDrone 1-10 to 0-9

                # Ignore ignored regions (category 0 or 11)
                if cat_id < 0 or cat_id > 9 or score == 0:
                    continue

                # Convert to YOLO center normalized coordinates
                x_center = max(0.0, min(1.0, (x + w / 2.0) / img_w))
                y_center = max(0.0, min(1.0, (y + h / 2.0) / img_h))
                norm_w = max(0.0, min(1.0, w / img_w))
                norm_h = max(0.0, min(1.0, h / img_h))

                yolo_lines.append(f"{cat_id} {x_center:.6f} {y_center:.6f} {norm_w:.6f} {norm_h:.6f}\n")

        label_out_path = os.path.join(labels_dir, filename)
        with open(label_out_path, 'w') as out_f:
            out_f.writelines(yolo_lines)

        converted_count += 1
        if converted_count % 1000 == 0:
            print(f"  • Converted {converted_count}/{len(ann_files)} files...")

    print(f"\n[SUCCESS] Converted {converted_count} VisDrone annotation files to YOLO format!")
    print(f"[SUCCESS] Saved to: {labels_dir}")
    print("===================================================================")

if __name__ == "__main__":
    convert_visdrone_to_yolo()
