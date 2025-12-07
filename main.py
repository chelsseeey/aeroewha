#!/usr/bin/env python3
"""
Drone Control System - Main Entry Point
"""

import asyncio
import logging
import yaml
import signal
import sys
from pathlib import Path

Path("logs").mkdir(parents=True, exist_ok=True)
# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler('logs/drone.log')
    ]
)

logger = logging.getLogger(__name__)


class DroneSystem:
    """Main drone control system"""
    
    def __init__(self, config_dir: str = "configs"):
        """Initialize drone system"""
        self.config_dir = Path(config_dir)
        self.configs = {}
        self.running = False
        
        # Subsystems
        self.video_stream = None
        self.detector = None
        self.tracker = None
        self.mavsdk_client = None
        self.controller = None
        self.state_machine = None
        self.safety_monitor = None
        self.data_recorder = None
        
        # Signal handling
        self.stop_event = asyncio.Event()
    
    def load_configs(self):
        """Load all configuration files"""
        logger.info("Loading configuration files...")
        
        camera_cfg = self.config_dir / "camera.yaml"
        control_cfg = self.config_dir / "control.yaml"
        system_cfg = self.config_dir / "system.yaml"
        
        if not camera_cfg.exists():
            raise FileNotFoundError(f"Camera config not found: {camera_cfg}")
        if not control_cfg.exists():
            raise FileNotFoundError(f"Control config not found: {control_cfg}")
        if not system_cfg.exists():
            raise FileNotFoundError(f"System config not found: {system_cfg}")
        
        with open(camera_cfg, 'r') as f:
            self.configs['camera'] = yaml.safe_load(f)
        with open(control_cfg, 'r') as f:
            self.configs['control'] = yaml.safe_load(f)
        with open(system_cfg, 'r') as f:
            self.configs['system'] = yaml.safe_load(f)
        
        logger.info("Configuration loaded successfully")
    
    async def initialize(self):
        """Initialize all subsystems"""
        logger.info("Initializing drone system...")
        
        try:
            # Import modules
            from vision.stream import VideoStream
            from vision.detector import ObjectDetector
            from vision.tracker import ObjectTracker
            from platform.mavsdk_client import MavsdkClient
            from control.controller import Controller
            from mission.state_machine import StateMachine, MissionState
            from platform.safety import SafetyMonitor
            from common.data_recorder import DataRecorder
            
            # Initialize Vision System
            logger.info("Initializing vision system...")
            camera_config = self.configs['camera']
            camera_hw = camera_config.get('camera', {})
            processing_cfg = camera_config.get('processing', {})
            
            self.video_stream = VideoStream(
                source_type=camera_hw.get('source_type', 'USB'),
                device_id=camera_hw.get('device_id', 0),
                video_path=camera_hw.get('video_path', ''),
                rtsp_url=camera_hw.get('rtsp_url', ''),
                width=camera_hw.get('resolution', {}).get('width', 1280),
                height=camera_hw.get('resolution', {}).get('height', 720),
                fps=camera_hw.get('resolution', {}).get('fps', 30),
                processing_cfg=processing_cfg,
            )
            
            # Initialize Object Detector (YOLO / Color)
            detection_config = camera_config.get('detection', {})
            method = detection_config.get('method', 'yolo')
            
            if method == 'yolo':
                yolo_config = detection_config.get('yolo', {})
                self.detector = ObjectDetector(
                    method='yolo',
                    model_path=yolo_config.get('model_path', 'yolov8n.pt'),
                    conf_threshold=yolo_config.get('conf_threshold', 0.5),
                    iou_threshold=yolo_config.get('iou_threshold', 0.45),
                    max_detections=yolo_config.get('max_detections', 10)
                )
                logger.info(f"YOLO detector initialized: {yolo_config.get('model_path')}")
            else:
                color_config = detection_config.get('color', {})
                self.detector = ObjectDetector(
                    method='color',
                    hsv_ranges=color_config.get('hsv_ranges', []),
                    min_area=color_config.get('min_area', 500)
                )
                logger.info("Color detector initialized")
            
            # Initialize Tracker
            self.tracker = ObjectTracker(tracker_type='KCF')
            logger.info("Object tracker initialized")
            
            # Initialize MAVSDK Client
            logger.info("Initializing MAVSDK client...")
            system_config = self.configs['system']
            
            # Pass the entire system section (includes mode, connection, etc.)
            self.mavsdk_client = MavsdkClient(system_config.get('system', system_config))
            # Note: Actual connection happens in run() to avoid blocking
            
            # Initialize Controller
            logger.info("Initializing controller...")
            control_config = self.configs['control']
            self.controller = Controller(control_config)
            
            # Initialize State Machine
            logger.info("Initializing state machine...")
            self.state_machine = StateMachine()  # No parameters needed
            
            # Initialize Safety Monitor
            logger.info("Initializing safety monitor...")
            safety_config = self.configs['system']['safety']
            home_position_ned = (0.0, 0.0, 0.0)  # Will be updated after takeoff
            self.safety_monitor = SafetyMonitor(safety_config, home_position_ned)
            
            # Initialize Data Recorder
            logger.info("Initializing data recorder...")
            logging_config = self.configs['system']['logging']
            if logging_config.get('enable', True):
                log_dir = logging_config.get('log_dir', 'logs/runs')
                self.data_recorder = DataRecorder(log_dir=log_dir)
            
            logger.info("✅ All subsystems initialized successfully!")
            
        except Exception as e:
            logger.error(f"Failed to initialize subsystems: {e}", exc_info=True)
            raise
    
    async def run(self):
        """Main run loop"""
        try:
            self.running = True
            logger.info("Starting main loop...")
            
            # Start video stream
            self.video_stream.open()
            
            logger.info("🎥 Video stream started")
            logger.info("🚁 Drone system is running...")
            logger.info("Press Ctrl+C to stop")
            
            frame_count = 0
            
            # TODO: connect to MAVSDK here (await self.mavsdk_client.connect())
            # TODO: start safety monitor here (await self.safety_monitor.start())
            
            while self.running:
                # Read frame
                try:
                    frame = self.video_stream.read()
                except Exception as e:
                    logger.warning(f"Failed to read frame: {e}")
                    await asyncio.sleep(0.05)
                    continue
                
                # Object detection (every frame or throttled)
                detections = self.detector.detect(frame)
                
                # Log detection results
                if detections:
                    for det in detections:
                        logger.debug(f"Detected: {det.class_name} ({det.confidence:.2f})")
                
                # TODO: Implement full control loop:
                # - Update state machine based on detections
                # - Compute control commands
                # - Send commands to drone via MAVSDK
                # - Update tracker
                # - Safety checks
                # - Data recording
                
                frame_count += 1
                if frame_count % 100 == 0:
                    logger.info(f"Processed {frame_count} frames, Detections: {len(detections)}")
                
                # Main loop rate: 20 Hz
                await asyncio.sleep(0.05)
                
        except Exception as e:
            logger.error(f"Error in main loop: {e}", exc_info=True)
            self.running = False
    
    async def shutdown(self):
        """Shutdown system gracefully"""
        logger.info("Shutting down system...")
        self.running = False
        
        try:
            # Stop video stream
            if self.video_stream:
                self.video_stream.release()
                logger.info("Video stream stopped")
            # Stop data recorder
            if self.data_recorder:
                self.data_recorder.stop()
                logger.info("Data recorder stopped")
            
            # Stop safety monitor
            if self.safety_monitor:
                await self.safety_monitor.stop()
                logger.info("Safety monitor stopped")
            
            # Disconnect MAVSDK
            if self.mavsdk_client:
                # Add disconnect logic if needed
                logger.info("MAVSDK client disconnected")
        
        except Exception as e:
            logger.error(f"Error during shutdown: {e}")
        
        logger.info("✅ System shutdown complete")
    
    def handle_signal(self, sig, frame):
        """Handle termination signals"""
        logger.info(f"Received signal: {sig}")
        self.running = False
        if not self.stop_event.is_set():
            self.stop_event.set()


async def main():
    """Main entry point"""
    system = DroneSystem(config_dir="configs")
    
    # Setup signal handlers
    signal.signal(signal.SIGINT, system.handle_signal)
    signal.signal(signal.SIGTERM, system.handle_signal)
    
    try:
        # Load configuration
        system.load_configs()
        
        # Initialize
        await system.initialize()
        
        # Run
        await system.run()
        
    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    except Exception as e:
        logger.error(f"Fatal error: {e}", exc_info=True)
    finally:
        await system.shutdown()
    
    logger.info("Drone system terminated")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
