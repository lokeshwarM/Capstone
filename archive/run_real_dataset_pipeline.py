import os
import glob
import time
import cv2  # pyfly: ignore [missing-import] # type: ignore
import numpy as np  # pyfly: ignore [missing-import] # type: ignore

from modules.perception import YOLOPerception
from modules.decision_engine import AIDecisionEngine

def main():
    print("===================================================================")
    print("  REAL DATASET IMAGE PIPELINE EXECUTION (VisDrone Dataset)")
    print("===================================================================")

    # Check both full extracted VisDrone path and standard images path
    base_dir = os.path.dirname(__file__)
    output_dir = os.path.join(base_dir, "results", "real_dataset_output")
    os.makedirs(output_dir, exist_ok=True)

    full_path_glob = glob.glob(os.path.join(base_dir, "data", "**", "images", "*.jpg"), recursive=True)
    
    if not full_path_glob:
        print("[Error] No .jpg images found inside d:/Capstone/data/")
        return

    # Pick up to 50 real images from the extracted dataset for evaluation
    image_paths = full_path_glob[:50]
    print(f"[Dataset] Found {len(full_path_glob)} total real images! Processing a batch of {len(image_paths)} images...")

    # Initialize Real Models
    perception = YOLOPerception(conf_threshold=0.60)
    engine = AIDecisionEngine(conf_threshold=0.60)

    print("\n--- PROCESSING REAL IMAGE FILES FROM DISK ---")

    for img_path in image_paths:
        filename = os.path.basename(img_path)
        print(f"\n[Image: {filename}] Reading file from disk...")

        # 1. Read real image from disk
        raw_img = cv2.imread(img_path)
        if raw_img is None:
            continue

        raw_bytes = os.path.getsize(img_path)
        raw_kb = raw_bytes / 1024.0

        # 2. Run real PyTorch YOLOv8 Object Detection
        start_time = time.time()
        boxes, confidences, Sk = perception.detect_objects(raw_img)
        inference_time_ms = (time.time() - start_time) * 1000.0

        print(f"  • Real Objects Detected : {len(boxes)} objects")
        print(f"  • Mean Detection Conf (Sk): {Sk:.4f}")
        print(f"  • PyTorch Inference Time : {inference_time_ms:.2f} ms")

        # 3. Apply Decision Engine Mode Selection (Paper 4)
        telemetry = {
            "confidence_Sk": Sk,
            "drift_variance": 4.5,
            "battery_soc": 75.0,
            "bandwidth_bk": 6.0,
            "drone_id": 1
        }
        actions = engine.evaluate_state_vector(telemetry)

        # 4. Perform Real Background Semantic Compression
        compressed_img, compressed_kb = perception.compress_background(raw_img, boxes=boxes, quality=25)
        
        savings_percent = ((raw_kb - compressed_kb) / raw_kb) * 100.0 if raw_kb > 0 else 0.0

        # 5. Draw bounding box visual annotations on real image
        vis_img = compressed_img.copy()
        for box in boxes:
            bx, by, bw_box, bh_box = [int(v) for v in box]
            x1, y1 = max(0, bx - bw_box // 2), max(0, by - bh_box // 2)
            x2, y2 = min(raw_img.shape[1], bx + bw_box // 2), min(raw_img.shape[0], by + bh_box // 2)
            cv2.rectangle(vis_img, (x1, y1), (x2, y2), (0, 255, 0), 2)

        out_path = os.path.join(output_dir, f"proc_{filename}")
        cv2.imwrite(out_path, vis_img)

        print(f"  • Comm Mode Decision   : {actions['comm_mode']}")
        print(f"  • Original File Size   : {raw_kb:.2f} KB")
        print(f"  • Compressed Payload   : {compressed_kb:.2f} KB")
        print(f"  • Measured Data Saved  : {savings_percent:.1f}% SAVED!")
        print(f"  • Output Image Saved   : {out_path}")

    print("\n===================================================================")
    print("  [SUCCESS] Real Dataset Image Pipeline Executed Cleanly!")
    print("===================================================================")

if __name__ == "__main__":
    main()
