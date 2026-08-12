import os
import cv2  # pyfly: ignore [missing-import] # type: ignore
import numpy as np  # pyfly: ignore [missing-import] # type: ignore


class YOLOPerception:
    """
    Perception Module based on YOLOv8 & Semantic Communication Mode Selection (Paper 4 - Du et al.).
    Handles object detection, confidence score (Sk) evaluation, and background compression.
    """
    def __init__(self, model_name="yolov8n.pt", conf_threshold=0.60):
        self.conf_threshold = conf_threshold
        # Check if custom trained VisDrone weights exist
        custom_weights = os.path.join(os.path.dirname(os.path.dirname(__file__)), "visdrone_yolov8_custom.pt")
        if os.path.exists(custom_weights):
            model_to_load = custom_weights
            print(f"[Perception] Loading custom fine-tuned VisDrone model ({custom_weights})...")
        else:
            model_to_load = model_name
            print(f"[Perception] Loading base YOLOv8 model ({model_name})...")
            
        self.model = None
        self.model_name = model_to_load
        self._init_model()

    def _init_model(self):
        try:
            from ultralytics import YOLO  # pyfly: ignore [missing-import] # type: ignore
            self.model = YOLO(self.model_name)
            print(f"[Perception] Loaded YOLOv8 model ({self.model_name}) successfully.")
        except Exception as e:
            print(f"[Perception Warning] Could not load ultralytics YOLO model: {e}")
            print("[Perception Warning] Running in simulated vision fallback mode.")

    def detect_objects(self, image_input):
        """
        Runs object detection on image input or frame.
        Returns:
            boxes: list of bounding boxes [x, y, w, h]
            confidences: list of confidence scores
            avg_confidence (Sk): aggregate detection confidence score
        """
        if self.model is not None and isinstance(image_input, np.ndarray):
            results = self.model(image_input, verbose=False)[0]
            boxes = []
            confidences = []
            for r in results.boxes:
                box = r.xywh[0].cpu().numpy().tolist()
                conf = float(r.conf[0].cpu().numpy())
                boxes.append(box)
                confidences.append(conf)
            
            Sk = float(np.mean(confidences)) if confidences else 0.0
            return boxes, confidences, Sk
        else:
            # Simulated fallback if no image array passed
            dummy_boxes = [[100, 100, 50, 50], [200, 150, 60, 40]]
            dummy_conf = [0.85, 0.78]
            Sk = float(np.mean(dummy_conf))
            return dummy_boxes, dummy_conf, Sk

    def select_compression_mode(self, Sk, raw_image=None):
        """
        Paper 4 Mode Selection Logic:
        If Sk >= conf_threshold: Mode 1 (Background compression, payload saved by ~70%)
        If Sk < conf_threshold: Mode 2 (Full raw frame sequence offload to Cloud BS)
        """
        if Sk >= self.conf_threshold:
            mode = "MODE_1_SEMANTIC_COMPRESSION"
            payload_size_kb = 35.0  # Approx compressed background + metadata
            compression_ratio = 0.3  # 70% data reduction
            compressed_img = None
            if isinstance(raw_image, np.ndarray):
                compressed_img, payload_size_kb = self.compress_background(raw_image, quality=25)
        else:
            mode = "MODE_2_FULL_OFFLOAD"
            payload_size_kb = 250.0  # Full HD raw image sequence
            compression_ratio = 1.0  # No reduction
            compressed_img = raw_image

        return {
            "mode": mode,
            "payload_kb": payload_size_kb,
            "compression_ratio": compression_ratio,
            "confidence_Sk": Sk,
            "compressed_image": compressed_img
        }

    def compress_background(self, image, boxes=[], quality=25):
        """
        Applies JPEG compression to background while keeping detected object regions intact.
        Returns:
            compressed_image (ndarray), size_kb (float)
        """
        h, w = image.shape[:2]
        # Encode background with lower quality
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
        _, enc_bg = cv2.imencode('.jpg', image, encode_param)
        compressed_bg = cv2.imdecode(enc_bg, 1)

        # Restore detected object bounding box regions to original high quality
        output_image = compressed_bg.copy()
        for box in boxes:
            bx, by, bw_box, bh_box = [int(v) for v in box]
            x1, y1 = max(0, bx - bw_box // 2), max(0, by - bh_box // 2)
            x2, y2 = min(w, bx + bw_box // 2), min(h, by + bh_box // 2)
            output_image[y1:y2, x1:x2] = image[y1:y2, x1:x2]

        size_kb = float(len(enc_bg)) / 1024.0
        return output_image, size_kb


class OpticalFlowTracker:
    """
    Cross-View Collaboration Module using Optical Flow & Homography (Paper 5 - Wu et al. CrossDrone).
    Tracks features using Lucas-Kanade (LK) optical flow and computes peer homography matrix H(A->B).
    """
    def __init__(self, drift_threshold=15.0):
        self.drift_threshold = drift_threshold
        # LK optical flow parameters
        self.lk_params = dict(
            winSize=(15, 15),
            maxLevel=2,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03)
        )

    def track_lk_optical_flow(self, old_gray, new_gray, p0=None):
        """
        Runs Lucas-Kanade Optical Flow (cv2.calcOpticalFlowPyrLK) to track keypoints across frames.
        Returns:
            p1: new tracked point coordinates
            st: status array (1 if found, 0 otherwise)
            var_dist: tracking spatial variance
        """
        if p0 is None or len(p0) == 0:
            # Detect good feature corners if initial points not provided
            p0 = cv2.goodFeaturesToTrack(old_gray, maxCorners=100, qualityLevel=0.3, minDistance=7, blockSize=7)

        if p0 is not None and len(p0) > 0:
            p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, new_gray, p0, None, **self.lk_params)
            good_new = p1[st == 1]
            good_old = p0[st == 1]
            var_dist = self.calculate_tracking_variance(good_old, good_new)
            return good_new, st, var_dist
        else:
            return np.array([]), np.array([]), 25.0

    def calculate_tracking_variance(self, prev_pts, curr_pts):
        """
        Calculates spatial tracking variance sigma_dist^2.
        """
        if prev_pts is None or curr_pts is None or len(prev_pts) == 0:
            return 25.0  # High drift fallback
        
        displacement = np.linalg.norm(curr_pts - prev_pts, axis=1)
        var_dist = float(np.var(displacement))
        return var_dist

    def compute_peer_homography(self, pts_drone_A, pts_drone_B):
        """
        Computes 3x3 Homography Transformation Matrix H(A->B) using RANSAC.
        Allows Peer Drone B to correct tracking drift using Drone A's view.
        """
        if len(pts_drone_A) >= 4 and len(pts_drone_B) >= 4:
            H, mask = cv2.findHomography(pts_drone_A, pts_drone_B, cv2.RANSAC, 5.0)
            return H, True
        else:
            # Fallback identity matrix
            return np.eye(3), False

    def evaluate_drift(self, var_dist):
        """
        Triggers Peer Cross-View Correction if tracking variance exceeds threshold.
        """
        is_drifted = var_dist > self.drift_threshold
        status = "TRIGGER_CROSS_VIEW_CORRECTION" if is_drifted else "TRACKING_STABLE"
        return is_drifted, status
