from modules.perception import YOLOPerception, OpticalFlowTracker
from modules.optimization import ACOAllocator, ALNSRouter

class AIDecisionEngine:
    """
    Event-Driven Hierarchical AI Decision Engine.
    Evaluates 4D State Vector S(t) = [Sk, sigma_dist^2, SoC_k, b_k] to trigger:
    1. Semantic Compression vs Raw Video Offloading (Paper 4)
    2. Optical Flow Cross-View Homography Drift Correction (Paper 5)
    3. ACO Task Allocation & ALNS Dynamic Re-routing (Paper 1 & Paper B)
    """
    def __init__(self, conf_threshold=0.65, drift_threshold=15.0, battery_threshold=20.0, bw_threshold=5.0):
        self.conf_threshold = conf_threshold
        self.drift_threshold = drift_threshold
        self.battery_threshold = battery_threshold
        self.bw_threshold = bw_threshold

        # Initialize Sub-modules
        self.perception = YOLOPerception(conf_threshold=conf_threshold)
        self.tracker = OpticalFlowTracker(drift_threshold=drift_threshold)
        self.aco_allocator = ACOAllocator()
        self.alns_router = ALNSRouter()

    def evaluate_state_vector(self, state_telemetry):
        """
        Input State Telemetry Dict:
            - 'confidence_Sk': float YOLO confidence score [0, 1]
            - 'drift_variance': float optical flow tracking variance
            - 'battery_soc': float drone battery percentage [0, 100]
            - 'bandwidth_bk': float wireless link speed in Mbps
            - 'drone_id': int drone identifier
        
        Returns decision actions dictionary.
        """
        Sk = state_telemetry.get('confidence_Sk', 0.8)
        drift_var = state_telemetry.get('drift_variance', 5.0)
        soc = state_telemetry.get('battery_soc', 80.0)
        bw = state_telemetry.get('bandwidth_bk', 8.0)
        drone_id = state_telemetry.get('drone_id', 0)

        actions = {}

        # 1. Perception & Communication Mode Selection
        comm_decision = self.perception.select_compression_mode(Sk)
        actions['comm_mode'] = comm_decision['mode']
        actions['payload_kb'] = comm_decision['payload_kb']

        # 2. Cross-View Homography Drift Correction
        is_drifted, drift_status = self.tracker.evaluate_drift(drift_var)
        actions['drift_status'] = drift_status
        actions['peer_correction_triggered'] = is_drifted

        # 3. Dynamic Route Management
        if soc < self.battery_threshold:
            actions['route_action'] = f"TRIGGER_ALNS_REROUTE_FOR_DRONE_{drone_id}"
            actions['battery_alert'] = True
        else:
            actions['route_action'] = "ACO_ROUTE_EXECUTION_NORMAL"
            actions['battery_alert'] = False

        return actions
