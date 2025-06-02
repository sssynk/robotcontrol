from Localization import Localization
from mdds30na import MDDS30AntiPhase
import math
import time
import threading

motor_control = MDDS30AntiPhase()

# Calibration offsets
position_offset = [0.0, 0.0, 0.0]  # x, y, z offsets
yaw_offset = 0.0

# Current calibrated position and yaw
current_position = [0.0, 0.0, 0.0]
current_yaw = 0.0

def handle_location_update(position, quaternion, euler_angles, is_tracking, battery_percent):
    global current_position, current_yaw
        
    if position and euler_angles:
        # Apply calibration offsets
        current_position = [
            position[0] - position_offset[0],
            position[1] - position_offset[1], 
            position[2] - position_offset[2]
        ]
        current_yaw = euler_angles[1] - yaw_offset  # yaw is index 1 in euler_angles
        
        
        
def calibrate():
    """Calibrate by setting current position to (0,0,0) and yaw to 0"""
    global position_offset, yaw_offset
        
    if current_position and current_yaw:
        position_offset = current_position.copy()
        yaw_offset = current_yaw  # yaw is index 1
        print("✓ Calibration complete! Position and yaw set to zero.")
        print(f"Position offset: {position_offset}")
        print(f"Yaw offset: {yaw_offset}")
    else:
        print("✗ Calibration failed: No position data available")

def normalize_angle(angle_deg):
    """Normalize angle to be within -180 to 180 degrees"""
    angle_deg = angle_deg % 360
    if angle_deg > 180:
        angle_deg -= 360
    elif angle_deg < -180:
        angle_deg += 360
    return angle_deg

def motor_command_threaded(func, *args):
    """Execute motor command in a separate thread to avoid blocking"""
    thread = threading.Thread(target=func, args=args, daemon=True)
    thread.start()

def move_to_direct(x, z):
    """Move to relative x,z location using turn-then-move approach"""
    print(f"Moving to relative position: x={x}, z={z}")
    print("Current calibrated position:", current_position)
    print("Current calibrated yaw:", current_yaw)
    
    # Target position in world coordinates
    target_x = x
    target_z = z
    
    # Current position
    current_x = current_position[0]
    current_z = current_position[2]  # z is index 2
    
    # Calculate displacement
    dx = target_x - current_x
    dz = target_z - current_z
    
    # Calculate target angle (bearing to target) - using dx, dz as mentioned
    target_angle = math.atan2(dx, dz)
    target_angle_deg = math.degrees(target_angle)
    
    print(f"Target angle: {target_angle_deg:.1f}°")
    
    # ===== PHASE 1: TURN TO TARGET =====
    # ===== PHASE 1: TURN TO TARGET =====
    print("Phase 1: Turning to target...")
    
    angle_tolerance = 2.0  # degrees - reduced from 5.0
    max_turn_speed = 0.5  # maximum turn speed
    min_turn_speed = 0.2  # minimum turn speed to avoid getting stuck
    kp = 0.01  # proportional gain - adjust this to tune responsiveness
    
    while True:
        # Get fresh data
        latest_data = localization.get_latest_data()
        if not latest_data['euler_angles']:
            print("No orientation data, stopping...")
            motor_command_threaded(motor_control.stop)
            return False
            
        # Update current yaw (already in degrees)
        current_yaw_fresh = latest_data['euler_angles'][1] - yaw_offset
        normalized_current_yaw = normalize_angle(current_yaw_fresh)
        normalized_target_angle = normalize_angle(target_angle_deg)
        
        # Calculate angle error
        angle_error = normalized_target_angle - normalized_current_yaw
        
        # Handle angle wrap-around
        if angle_error > 180:
            angle_error -= 360
        elif angle_error < -180:
            angle_error += 360
            
        print(f"Angle error: {angle_error:.1f}°")
        
        # Check if we're close enough
        if abs(angle_error) < angle_tolerance:
            print("✓ Angle reached!")
            motor_command_threaded(motor_control.stop)
            time.sleep(0.5)  # brief pause
            break
            
        # Calculate proportional turn speed based on angle error
        turn_speed = abs(angle_error) * kp
        
        # Clamp turn speed between min and max
        turn_speed = max(min_turn_speed, min(turn_speed, max_turn_speed))
        
        print(f"Turn speed: {turn_speed:.3f}")
        
        # Turn towards target - THREADED
        if angle_error > 0:
            # Turn left (counter-clockwise) - SWAPPED
            motor_command_threaded(motor_control.set, turn_speed, -turn_speed)
        else:
            # Turn right (clockwise) - SWAPPED
            motor_command_threaded(motor_control.set, -turn_speed, turn_speed)
            
        time.sleep(0.1)  # Control loop delay
    
    # ===== PHASE 2: MOVE TO TARGET =====
    print("Phase 2: Moving to target...")
    
    distance_tolerance = 0.25  # meters (15cm)
    max_forward_speed = 0.6  # maximum forward speed
    min_forward_speed = 0.15  # minimum forward speed to avoid getting stuck
    distance_kp = 0.8  # proportional gain for distance-based speed control
    
    while True:
        # Get fresh position data
        latest_data = localization.get_latest_data()
        if not latest_data['position']:
            print("No position data, stopping...")
            motor_command_threaded(motor_control.stop)
            return False
        
        # Update current position
        fresh_position = [
            latest_data['position'][0] - position_offset[0],
            latest_data['position'][1] - position_offset[1],
            latest_data['position'][2] - position_offset[2]
        ]
        
        # Calculate distance to target
        distance = math.sqrt((target_x - fresh_position[0])**2 + 
                           (target_z - fresh_position[2])**2)
        
        print(f"Distance to target: {distance:.2f}m")
        
        # Check if we've reached the target
        if distance < distance_tolerance:
            print("✓ Target reached!")
            motor_command_threaded(motor_control.stop)
            return True
        
        # Calculate base forward speed proportional to distance
        base_forward_speed = distance * distance_kp
        
        # Clamp forward speed between min and max
        base_forward_speed = max(min_forward_speed, min(base_forward_speed, max_forward_speed))
        
        print(f"Base forward speed: {base_forward_speed:.3f}")
        
        # Calculate displacement for angle correction
        dx = target_x - fresh_position[0]
        dz = target_z - fresh_position[2]
        
        # Calculate target angle (bearing to target)
        target_angle = math.atan2(dx, dz)
        target_angle_deg = math.degrees(target_angle)
        
        current_yaw_fresh = latest_data['euler_angles'][1] - yaw_offset
        normalized_current_yaw = normalize_angle(current_yaw_fresh)
        normalized_target_angle = normalize_angle(target_angle_deg)
        
        # Calculate angle error
        angle_error = normalized_target_angle - normalized_current_yaw
        
        # Handle angle wrap-around
        if angle_error > 180:
            angle_error -= 360
        elif angle_error < -180:
            angle_error += 360
            
        print(f"Angle error: {angle_error:.1f}°")
        
        # Apply angle correction to the base forward speed
        left_speed = base_forward_speed - angle_error * kp
        right_speed = base_forward_speed + angle_error * kp
        
        print(f"Left speed: {left_speed:.3f}, Right speed: {right_speed:.3f}")
            
        # Move forward with proportional speed - THREADED
        motor_command_threaded(motor_control.set, right_speed, left_speed)
        time.sleep(0.1)  # Control loop delay

def move_to_bu(x, z):
    """Move to relative x,z location using turn-then-move approach"""
    print(f"Moving to relative position: x={x}, z={z}")
    print("Current calibrated position:", current_position)
    print("Current calibrated yaw:", current_yaw)
    
    # Target position in world coordinates
    target_x = x
    target_z = z
    
    # Current position
    current_x = current_position[0]
    current_z = current_position[2]  # z is index 2
    
    # Calculate displacement
    dx = target_x - current_x
    dz = target_z - current_z
    
    # Calculate target angle (bearing to target) - using dx, dz as mentioned
    target_angle = math.atan2(dx, dz)
    target_angle_deg = math.degrees(target_angle)
    
    print(f"Target angle: {target_angle_deg:.1f}°")
    
    # ===== PHASE 1: TURN TO TARGET =====
    # ===== PHASE 1: TURN TO TARGET =====
    print("Phase 1: Turning to target...")
    
    angle_tolerance = 360.0  # degrees - reduced from 5.0
    max_turn_speed = 0.5  # maximum turn speed
    min_turn_speed = 0.2  # minimum turn speed to avoid getting stuck
    kp = 0.01  # proportional gain - adjust this to tune responsiveness
    
    while True:
        # Get fresh data
        latest_data = localization.get_latest_data()
        if not latest_data['euler_angles']:
            print("No orientation data, stopping...")
            motor_command_threaded(motor_control.stop)
            return False
            
        # Update current yaw (already in degrees)
        current_yaw_fresh = latest_data['euler_angles'][1] - yaw_offset
        normalized_current_yaw = normalize_angle(current_yaw_fresh)
        normalized_target_angle = normalize_angle(target_angle_deg)
        
        # Calculate angle error
        angle_error = normalized_target_angle - normalized_current_yaw
        
        # Handle angle wrap-around
        if angle_error > 180:
            angle_error -= 360
        elif angle_error < -180:
            angle_error += 360
            
        print(f"Angle error: {angle_error:.1f}°")
        
        # Check if we're close enough
        if abs(angle_error) < angle_tolerance:
            print("✓ Angle reached!")
            motor_command_threaded(motor_control.stop)
            time.sleep(0.5)  # brief pause
            break
            
        # Calculate proportional turn speed based on angle error
        turn_speed = abs(angle_error) * kp
        
        # Clamp turn speed between min and max
        turn_speed = max(min_turn_speed, min(turn_speed, max_turn_speed))
        
        print(f"Turn speed: {turn_speed:.3f}")
        
        # Turn towards target - THREADED
        if angle_error > 0:
            # Turn left (counter-clockwise) - SWAPPED
            motor_command_threaded(motor_control.set, turn_speed, -turn_speed)
        else:
            # Turn right (clockwise) - SWAPPED
            motor_command_threaded(motor_control.set, -turn_speed, turn_speed)
            
        time.sleep(0.1)  # Control loop delay
    
    # ===== PHASE 2: MOVE TO TARGET =====
    print("Phase 2: Moving to target...")
    
    distance_tolerance = 0.25  # meters (15cm)
    max_forward_speed = 0.4  # maximum forward speed
    min_forward_speed = 0.15  # minimum forward speed to avoid getting stuck
    distance_kp = 0.8  # proportional gain for distance-based speed control
    
    while True:
        # Get fresh position data
        latest_data = localization.get_latest_data()
        if not latest_data['position']:
            print("No position data, stopping...")
            motor_command_threaded(motor_control.stop)
            return False
        
        # Update current position
        fresh_position = [
            latest_data['position'][0] - position_offset[0],
            latest_data['position'][1] - position_offset[1],
            latest_data['position'][2] - position_offset[2]
        ]
        
        # Calculate distance to target
        distance = math.sqrt((target_x - fresh_position[0])**2 + 
                           (target_z - fresh_position[2])**2)
        
        print(f"Distance to target: {distance:.2f}m")
        
        # Check if we've reached the target
        if distance < distance_tolerance:
            print("✓ Target reached!")
            motor_command_threaded(motor_control.stop)
            return True
        
        # Calculate base forward speed proportional to distance
        base_forward_speed = distance * distance_kp
        
        # Clamp forward speed between min and max
        base_forward_speed = max(min_forward_speed, min(base_forward_speed, max_forward_speed))
        
        print(f"Base forward speed: {base_forward_speed:.3f}")
        
        # Calculate displacement for angle correction
        dx = target_x - fresh_position[0]
        dz = target_z - fresh_position[2]
        
        # Calculate target angle (bearing to target)
        target_angle = math.atan2(dx, dz)
        target_angle_deg = math.degrees(target_angle)
        
        current_yaw_fresh = latest_data['euler_angles'][1] - yaw_offset
        normalized_current_yaw = normalize_angle(current_yaw_fresh)
        normalized_target_angle = normalize_angle(target_angle_deg)
        
        # Calculate angle error
        angle_error = normalized_target_angle - normalized_current_yaw
        
        # Handle angle wrap-around
        if angle_error > 180:
            angle_error -= 360
        elif angle_error < -180:
            angle_error += 360
            
        print(f"Angle error: {angle_error:.1f}°")
        
        # Apply angle correction to the base forward speed
        left_speed = base_forward_speed - angle_error * kp
        right_speed = base_forward_speed + angle_error * kp
        
        print(f"Left speed: {left_speed:.3f}, Right speed: {right_speed:.3f}")
            
        # Move forward with proportional speed - THREADED
        motor_command_threaded(motor_control.set, right_speed, left_speed)
        time.sleep(0.1)  # Control loop delay

def move_to(x, z):
    """Move to relative x,z location using turn-then-move approach"""
    print(f"Moving to relative position: x={x}, z={z}")
    print("Current calibrated position:", current_position)
    print("Current calibrated yaw:", current_yaw)

    # ───────────── TARGET & INITIAL STATE ─────────────
    target_x, target_z = x, z
    current_x, current_z = current_position[0], current_position[2]
    dx, dz = target_x - current_x, target_z - current_z
    target_angle_deg = math.degrees(math.atan2(dx, dz))
    print(f"Target angle: {target_angle_deg:.1f}°")

    # ===== PHASE 1: TURN TO TARGET (unchanged) =====
    print("Phase 1: Turning to target...")
    angle_tolerance = 360.0
    max_turn_speed, min_turn_speed = 0.5, 0.2
    kp = 0.01

    while True:
        latest_data = localization.get_latest_data()
        if not latest_data['euler_angles']:
            print("No orientation data, stopping...")
            motor_command_threaded(motor_control.stop)
            return False

        current_yaw_fresh = latest_data['euler_angles'][1] - yaw_offset
        normalized_current_yaw = normalize_angle(current_yaw_fresh)
        normalized_target_angle = normalize_angle(target_angle_deg)
        angle_error = normalized_target_angle - normalized_current_yaw
        if angle_error > 180:
            angle_error -= 360
        elif angle_error < -180:
            angle_error += 360
        print(f"Angle error: {angle_error:.1f}°")

        if abs(angle_error) < angle_tolerance:
            print("✓ Angle reached!")
            motor_command_threaded(motor_control.stop)
            time.sleep(0.5)
            break

        turn_speed = max(min_turn_speed, min(abs(angle_error) * kp, max_turn_speed))
        print(f"Turn speed: {turn_speed:.3f}")

        if angle_error > 0:
            motor_command_threaded(motor_control.set,  turn_speed, -turn_speed)  # left
        else:
            motor_command_threaded(motor_control.set, -turn_speed,  turn_speed)  # right

        time.sleep(0.1)

    # ===== PHASE 2: MOVE TO TARGET (accelerates smoothly) =====
    print("Phase 2: Moving to target...")

    distance_tolerance = 0.15           # m
    max_forward_speed = 0.6             # cap
    min_forward_speed = 0.3            # to avoid stall
    distance_kp       = 0.4             # proportional factor
    accel_step        = 0.02            # speed increment every loop
    current_fwd_speed = 0.0             # starts from rest
    kp = 0.003

    while True:
        latest_data = localization.get_latest_data()
        if not latest_data['position']:
            print("No position data, stopping...")
            motor_command_threaded(motor_control.stop)
            return False

        # fresh position
        px = latest_data['position'][0] - position_offset[0]
        pz = latest_data['position'][2] - position_offset[2]

        distance = math.hypot(target_x - px, target_z - pz)
        print(f"Distance to target: {distance:.2f} m")

        if distance < distance_tolerance:
            print("✓ Target reached!")
            motor_command_threaded(motor_control.stop)
            return True

        # desired forward speed based on distance
        desired_speed = max(min_forward_speed,
                            min(distance * distance_kp, max_forward_speed))

        # accelerate smoothly toward desired_speed
        if current_fwd_speed < desired_speed:
            current_fwd_speed = min(current_fwd_speed + accel_step, desired_speed)
        else:  # decel stays immediate (already provided by proportional rule)
            current_fwd_speed = desired_speed

        print(f"Current forward speed: {current_fwd_speed:.3f}")

        # bearing correction
        dx, dz = target_x - px, target_z - pz
        target_angle_deg = math.degrees(math.atan2(dx, dz))

        current_yaw_fresh = latest_data['euler_angles'][1] - yaw_offset
        nc_yaw = normalize_angle(current_yaw_fresh)
        nt_angle = normalize_angle(target_angle_deg)
        angle_error = nt_angle - nc_yaw
        if angle_error > 180:
            angle_error -= 360
        elif angle_error < -180:
            angle_error += 360
        print(f"Angle error: {angle_error:.1f}°")

        left_speed  = current_fwd_speed - angle_error * kp
        right_speed = current_fwd_speed + angle_error * kp
        print(f"Left speed: {left_speed:.3f}, Right speed: {right_speed:.3f}")

        motor_command_threaded(motor_control.set, right_speed, left_speed)
        time.sleep(0.1)

def print_help():
    """Print available commands"""
    print("\nAvailable commands:")
    print("  calibrate           - Set current position to (0,0,0) and yaw to 0")
    print("  move_to <x> <z>     - Move to relative x,z position")
    print("  status              - Show current position and tracking status")
    print("  help                - Show this help message")
    print("  quit                - Exit the program")

def show_status():
    """Show current robot status"""
    latest_data = localization.get_latest_data()
    print("\n=== Robot Status ===")
    print(f"Raw Position: {latest_data['position']}")
    print(f"Calibrated Position: {current_position}")
    print(f"Raw Yaw: {latest_data['euler_angles'][1] if latest_data['euler_angles'] else 'N/A'}")
    print(f"Calibrated Yaw: {current_yaw}")
    print(f"Is Tracking: {latest_data['is_tracking']}")
    print(f"Battery: {latest_data['battery_percent']}%")

def command_loop():
    """Main command loop for user interaction"""
    print("🤖 Robot Path Following System")
    print("Type 'help' for available commands")
    
    while True:
        try:
            command = input("\nrobot> ").strip().lower()
            
            if not command:
                continue
            
            parts = command.split()
            cmd = parts[0]
            
            if cmd == "quit" or cmd == "exit":
                print("Goodbye!")
                break
            elif cmd == "help":
                print_help()
            elif cmd == "calibrate":
                calibrate()
            elif cmd == "status":
                show_status()
            elif cmd == "move_to":
                if len(parts) != 3:
                    print("Usage: move_to <x> <z>")
                    print("Example: move_to 1.5 -2.0")
                else:
                    try:
                        x = float(parts[1])
                        z = float(parts[2])
                        move_to(x, z)
                    except ValueError:
                        print("Error: x and z must be numbers")
            else:
                print(f"Unknown command: {cmd}")
                print("Type 'help' for available commands")
                
        except KeyboardInterrupt:
            print("\nGoodbye!")
            break
        except Exception as e:
            print(f"Error: {e}")

localization = Localization(handle_location_update)

# Start the command loop instead of running localization forever
if __name__ == "__main__":
    try:
        # Give localization a moment to start up
        import time
        time.sleep(1)
        command_loop()
    finally:
        motor_control.close()

# list of commands, calibrate is one which sets position to 0,0,0 and sets the yaw to 0 also

