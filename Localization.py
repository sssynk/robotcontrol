#!/usr/bin/env python3
import time
from ntcore import NetworkTableInstance, EventFlags, Topic

class Localization:
    """
    A NetworkTables server that receives QuestNav data and provides a single callback
    with all the most recent data whenever any value updates.
    """
    
    def __init__(self, callback_func, nt4_port=5810, nt3_port=1735):
        """
        Initialize the Localization server.
        
        Args:
            callback_func: Function to call when data updates. Will receive:
                (position, quaternion, euler_angles, is_tracking, battery_percent)
            nt4_port: NT4 port (default: 5810)
            nt3_port: NT3 port (default: 1735, set 0 to disable)
        """
        self.callback_func = callback_func
        
        # Store the latest values
        self.latest_position = None
        self.latest_quaternion = None
        self.latest_euler_angles = None
        self.latest_is_tracking = None
        self.latest_battery_percent = None
        
        # Network prefixes to listen to
        self.PREFIXES = [
            "/questnav/position", 
            "/questnav/quaternion", 
            "/questnav/eulerAngles",
            "/questnav/device/isTracking",
            "/questnav/device/batteryPercent",
        ]
        
        # Initialize NetworkTables
        self.inst = NetworkTableInstance.getDefault()
        self.inst.startServer(
            persist_filename="networktables.ini",
            listen_address="0.0.0.0",
            port3=nt3_port,
            port4=nt4_port,
        )
        
        # Set up the listener
        self.inst.addListener(
            self.PREFIXES,
            EventFlags.kValueAll | EventFlags.kImmediate,
            self._on_event
        )
        
        print(f"Wilson Precision Localization NT4 server listening on {nt4_port}")
    
    def _on_event(self, ev):
        """Internal event handler that updates latest values and calls user callback."""
        topic_name = ev.data.topic.getName()
        
        # Update the appropriate latest value
        if topic_name == "/questnav/position":
            self.latest_position = ev.data.value.getFloatArray()
        elif topic_name == "/questnav/quaternion":
            self.latest_quaternion = ev.data.value.getFloatArray()
        elif topic_name == "/questnav/eulerAngles":
            self.latest_euler_angles = ev.data.value.getFloatArray()
        elif topic_name == "/questnav/device/isTracking":
            self.latest_is_tracking = ev.data.value.getBoolean()
        elif topic_name == "/questnav/device/batteryPercent":
            self.latest_battery_percent = ev.data.value.getDouble()
        
        # Call the user's callback with all the latest data
        self.callback_func(
            position=self.latest_position,
            quaternion=self.latest_quaternion,
            euler_angles=self.latest_euler_angles,
            is_tracking=self.latest_is_tracking,
            battery_percent=self.latest_battery_percent
        )
    
    def get_latest_data(self):
        """
        Get all the latest data as a dictionary.
        
        Returns:
            dict: Dictionary containing all latest values
        """
        return {
            'position': self.latest_position,
            'quaternion': self.latest_quaternion,
            'euler_angles': self.latest_euler_angles,
            'is_tracking': self.latest_is_tracking,
            'battery_percent': self.latest_battery_percent
        }
    
    def run_forever(self):
        """Run the server indefinitely."""
        try:
            while True:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nShutting down QuestNav server...")


# Example usage:
if __name__ == "__main__":
    def my_callback(position, quaternion, euler_angles, is_tracking, battery_percent):
        """Example callback function that prints all received data."""
        print("=== QuestNav Data Update ===")
        print(f"Position: {position}")
        print(f"Quaternion: {quaternion}")
        print(f"Euler Angles: {euler_angles}")
        print(f"Is Tracking: {is_tracking}")
        print(f"Battery: {battery_percent}%")
        print()
    
    # Create and run the localization server
    localization = Localization(my_callback)
    localization.run_forever() 
