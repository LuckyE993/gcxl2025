import cv2
import time
import os
from datetime import datetime

class CameraController:
    """
    Camera controller class for capturing images at specified intervals
    with configurable camera parameters.
    """
    
    def __init__(self, camera_id=0, resolution=(640, 480), fps=30):
        """
        Initialize camera controller with specified parameters
        
        Args:
            camera_id (int): ID of the camera to use (default: 0 for first camera)
            resolution (tuple): Width and height resolution (default: 640x480)
            fps (int): Frames per second (default: 30)
        """
        self.camera_id = camera_id
        self.resolution = resolution
        self.fps = fps
        self.camera = None
        self.is_running = False
    
    def start(self):
        """Start camera capture"""
        self.camera = cv2.VideoCapture(self.camera_id)
        
        # Set camera properties
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])
        self.camera.set(cv2.CAP_PROP_FPS, self.fps)
        
        if not self.camera.isOpened():
            raise ValueError(f"Could not open camera with ID {self.camera_id}")
        
        self.is_running = True
        return True
    
    def stop(self):
        """Stop camera capture"""
        if self.camera and self.is_running:
            self.camera.release()
            self.is_running = False
        cv2.destroyAllWindows()
    
    def capture_single(self, show_preview=True):
        """
        Capture a single frame from the camera
        
        Args:
            show_preview (bool): Whether to display preview window
            
        Returns:
            numpy.ndarray: Image data if capture successful, None otherwise
        """
        if not self.is_running:
            self.start()
            
        ret, frame = self.camera.read()
        
        if ret:
            if show_preview:
                cv2.imshow('Camera Preview', frame)
                cv2.waitKey(1)
            return frame
        return None
    
    def capture_interval(self, interval, duration=None, save_dir='captures', show_preview=True):
        """
        Capture images at specified intervals
        
        Args:
            interval (float): Time interval between captures in seconds
            duration (float): Total duration to capture in seconds (None for unlimited)
            save_dir (str): Directory to save captured images
            show_preview (bool): Whether to display preview window
        """
        if not self.is_running:
            self.start()
        
        # Create save directory if it doesn't exist
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
            
        start_time = time.time()
        last_capture_time = start_time
        count = 0
        
        print(f"Starting capture at {interval}s intervals")
        print("Press 'q' to stop capture")
        
        try:
            while True:
                current_time = time.time()
                elapsed = current_time - start_time
                
                # Check if duration limit reached
                if duration is not None and elapsed >= duration:
                    print(f"Capture duration ({duration}s) reached. Stopping.")
                    break
                
                # Check if interval elapsed
                if current_time - last_capture_time >= interval:
                    frame = self.capture_single(show_preview=False)
                    
                    if frame is not None:
                        # Generate filename with timestamp
                        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                        filename = os.path.join(save_dir, f"img_{timestamp}.jpg")
                        cv2.imwrite(filename, frame)
                        count += 1
                        print(f"Captured image {count}: {filename}")
                        last_capture_time = current_time
                    
                # Show preview if requested
                if show_preview:
                    ret, frame = self.camera.read()
                    if ret:
                        cv2.imshow('Camera Preview', frame)
                
                # Check for quit key
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("Capture stopped by user")
                    break
                
                # Small sleep to prevent high CPU usage
                time.sleep(0.01)
                
        finally:
            self.stop()
            print(f"Capture complete. {count} images saved to {save_dir}")


def list_available_cameras():
    """
    List all available camera devices
    
    Returns:
        list: List of available camera indices
    """
    available_cameras = []
    for i in range(10):  # Check first 10 camera indices
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            available_cameras.append(i)
            cap.release()
    return available_cameras


if __name__ == "__main__":
    # List available cameras
    cameras = list_available_cameras()
    if not cameras:
        print("No cameras found!")
        exit(1)
    
    print(f"Available cameras: {cameras}")
    
    # Get user inputs
    camera_id = int(input(f"Select camera ID (0-{max(cameras)}): ") or "0")
    width = int(input("Enter width resolution (default: 640): ") or "640")
    height = int(input("Enter height resolution (default: 480): ") or "480")
    fps = int(input("Enter frame rate (default: 30): ") or "30")
    interval = float(input("Enter capture interval in seconds (default: 0.2): ") or "0.2")
    duration = input("Enter total capture duration in seconds (leave empty for unlimited): ")
    
    # Add save path customization
    default_save_path = f"captures_{camera_id}_{width}x{height}"
    save_path = input(f"Enter save directory path (default: {default_save_path}): ") or default_save_path
    
    duration = float(duration) if duration else None
    
    # Initialize camera controller
    controller = CameraController(
        camera_id=camera_id,
        resolution=(width, height),
        fps=fps
    )
    
    # Start capture
    try:
        controller.capture_interval(
            interval=interval,
            duration=duration,
            save_dir=save_path
        )
    except KeyboardInterrupt:
        print("Capture interrupted by user")
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        controller.stop()
