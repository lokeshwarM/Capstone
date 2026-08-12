import os
import sys

def train_visdrone_model(epochs=10, batch_size=16, img_size=640):
    """
    Fine-tunes PyTorch YOLOv8 on your extracted VisDrone dataset using your NVIDIA RTX 2050 GPU.
    Saves trained weights to d:/Capstone/visdrone_yolov8_custom.pt
    """
    print("===================================================================")
    print("  FINE-TUNING YOLOV8 ON FULL VISDRONE AERIAL DATASET")
    print("===================================================================")

    try:
        from ultralytics import YOLO
    except ImportError:
        print("[Error] ultralytics package not installed. Run: pip install ultralytics")
        return

    yaml_config = os.path.join(os.path.dirname(__file__), "data", "visdrone.yaml")
    if not os.path.exists(yaml_config):
        print(f"[Error] Config file not found at {yaml_config}")
        return

    print(f"[Training Setup] Dataset Config : {yaml_config}")
    print(f"[Training Setup] Training Epochs : {epochs}")
    print(f"[Training Setup] Batch Size     : {batch_size}")
    print(f"[Training Setup] Image Size     : {img_size}x{img_size}")
    print(f"[Training Setup] Target Weights : d:/Capstone/visdrone_yolov8_custom.pt")
    print("\n[Training Started] Training PyTorch YOLOv8 on your GPU. Please wait...")

    import torch
    device_setting = 0 if torch.cuda.is_available() else 'cpu'
    print(f"[Training Setup] Hardware Device : {device_setting} (CUDA available: {torch.cuda.is_available()})")

    # Load base model
    model = YOLO("yolov8n.pt")

    # Start training
    results = model.train(
        data=yaml_config,
        epochs=epochs,
        batch=batch_size,
        imgsz=img_size,
        device=device_setting,
        project="results/runs",
        name="visdrone_yolov8",
        exist_ok=True
    )

    # Save best custom weights to project root
    best_weights_path = os.path.join("results", "runs", "visdrone_yolov8", "weights", "best.pt")
    custom_target_path = os.path.join(os.path.dirname(__file__), "visdrone_yolov8_custom.pt")

    if os.path.exists(best_weights_path):
        import shutil
        shutil.copy(best_weights_path, custom_target_path)
        print(f"\n[SUCCESS] Custom VisDrone Model Trained Successfully!")
        print(f"[SUCCESS] Trained Weights Saved To: {custom_target_path}")
    else:
        print(f"\n[Training Complete] Trained weights generated in results/runs/visdrone_yolov8/")

    print("\n===================================================================")
    print("  TRAINING COMPLETE! PERCEPTION MODULE IS NOW VISDRONE FINE-TUNED")
    print("===================================================================")

if __name__ == "__main__":
    # Run 10 training epochs
    train_visdrone_model(epochs=10, batch_size=16, img_size=640)
