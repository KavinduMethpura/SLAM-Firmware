import socket
import sys
import threading
import time

# Configurations (Must match config.h)
UDP_LISTEN_PORT = 5005  # Port where Python receives data from ESP32
UDP_SEND_PORT = 5006    # Port where ESP32 listens for commands

# Shared state
robot_ip = None
is_running = False
start_time = None
state_lock = threading.Lock()

def print_command_menu():
    print("\n================== COMMAND PANEL ==================")
    print("Available Commands:")
    print("  'I'              - Start the Algorithm")
    print("  'R'              - Reset Odometry and Sensor Fusion")
    print("  'C'              - Calibrate Gyro (keep robot static)")
    print("  'S'              - Emergency Stop")
    print("  'M'              - Enter Manual Keyboard Control Mode (WASD)")
    print("  'K,left,right'   - Set motor calibration scales (e.g. K,1.0,0.85)")
    print("  'V,linear,yaw'   - Send custom velocities (e.g. V,0.2,-0.4)")
    print("  Ctrl+C           - Exit program")
    print("===================================================\n")

def listen_telemetry():
    global robot_ip, is_running, start_time
    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    try:
        sock.bind(("0.0.0.0", UDP_LISTEN_PORT))
        print(f"[INFO] Listening for robot telemetry on UDP port {UDP_LISTEN_PORT}...")
    except Exception as e:
        print(f"[ERROR] Failed to bind to port {UDP_LISTEN_PORT}: {e}")
        sys.exit(1)
        
    last_imu_print = time.time()

    while True:
        try:
            data, addr = sock.recvfrom(1024)
            with state_lock:
                if robot_ip != addr[0]:
                    robot_ip = addr[0]
                    print(f"\n[INFO] Connected to robot at IP: {robot_ip}")
            
            message = data.decode("utf-8").strip()
            
            # Check for 30s timeout
            with state_lock:
                if is_running and start_time is not None:
                    if time.time() - start_time >= 30.0:
                        is_running = False
                        start_time = None
                        # Send stop command to robot
                        if robot_ip is not None:
                            try:
                                stop_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                                payload = "C,S"
                                stop_sock.sendto(payload.encode("utf-8"), (robot_ip, UDP_SEND_PORT))
                                print(f"\n[TIMEOUT] 30 seconds elapsed. Algorithm stopped. Sent EMERGENCY STOP to robot.")
                            except Exception as e:
                                print(f"\n[ERROR] Failed to send stop command: {e}")
                        # Print the command panel menu again
                        print_command_menu()
            
            with state_lock:
                should_print = is_running
                
            if not should_print:
                continue
            
            # Parse message types
            if message.startswith("$ODOM"):
                # Format: $ODOM,x,y,theta,linear_vel,angular_vel
                parts = message.split(",")
                if len(parts) == 6:
                    x, y, theta, l_vel, a_vel = map(float, parts[1:])
                    print(f"[POSE] Fused: X: {x:6.3f} m | Y: {y:6.3f} m | Heading: {theta:6.3f} rad | Vel: {l_vel:5.2f} m/s, {a_vel:5.2f} rad/s")
                    
            elif message.startswith("$IMUR"):
                # Format: $IMUR,ax,ay,az,gx,gy,gz
                parts = message.split(",")
                if len(parts) == 7:
                    ax, ay, az, gx, gy, gz = map(float, parts[1:])
                    # Rate limit IMU printing to every 1 second to keep console clean
                    if time.time() - last_imu_print >= 1.0:
                        print(f"[IMU ] Accel: ({ax:5.2f}, {ay:5.2f}, {az:5.2f}) m/s² | Gyro: ({gx:5.2f}, {gy:5.2f}, {gz:5.2f}) rad/s")
                        last_imu_print = time.time()
                        
            elif message.startswith("$SCAN"):
                # Format: $SCAN,angle,distance_mm
                parts = message.split(",")
                if len(parts) == 3:
                    angle = int(parts[1])
                    distance = int(parts[2])
                    # Simple text-based bar graph representing range
                    bar_length = min(distance, 1000) // 40
                    bar = "=" * bar_length
                    print(f"[SCAN] Angle: {angle:3d}° | Distance: {distance:4d} mm | {bar}")
                    
            elif message.startswith("$DEBUG"):
                print(f"[ROBOT DEBUG] {message[7:]}")
                    
        except Exception as e:
            # Catch decode or parsing errors gracefully
            pass

def manual_control_mode(send_sock, target_ip):
    import msvcrt
    print("\n================ MAN-MACHINE CONTROL BOARD ================")
    print("Use the following keys to drive the robot:")
    print("  'W' - Move Forward")
    print("  'S' - Move Backward")
    print("  'A' - Turn Left")
    print("  'D' - Turn Right")
    print("  'Space' or 'X' - Stop")
    print("  'Q' - Exit Manual Control Mode")
    print("===========================================================\n")
    
    linear_vel = 0.0
    angular_vel = 0.0
    
    # Speed increments
    LINEAR_STEP = 0.05
    ANGULAR_STEP = 0.1
    MAX_LINEAR = 0.4
    MAX_ANGULAR = 0.8
    
    while True:
        # Check if a key was pressed
        if msvcrt.kbhit():
            try:
                char_byte = msvcrt.getch()
                key = char_byte.decode("utf-8", errors="ignore").lower()
            except Exception:
                continue

            if key == 'q':
                print("[INFO] Exiting Manual Control Mode.")
                # Send stop velocity before exit
                payload = "V,0.0,0.0"
                send_sock.sendto(payload.encode("utf-8"), (target_ip, UDP_SEND_PORT))
                break
            elif key == 'w':
                linear_vel = round(min(linear_vel + LINEAR_STEP, MAX_LINEAR), 2)
            elif key == 's':
                linear_vel = round(max(linear_vel - LINEAR_STEP, -MAX_LINEAR), 2)
            elif key == 'a':
                angular_vel = round(min(angular_vel + ANGULAR_STEP, MAX_ANGULAR), 2)
            elif key == 'd':
                angular_vel = round(max(angular_vel - ANGULAR_STEP, -MAX_ANGULAR), 2)
            elif key == ' ' or key == 'x':
                linear_vel = 0.0
                angular_vel = 0.0
            else:
                continue
                
            payload = f"V,{linear_vel},{angular_vel}"
            send_sock.sendto(payload.encode("utf-8"), (target_ip, UDP_SEND_PORT))
            print(f"[MANUAL] Sent: {payload} | Linear: {linear_vel:.2f} m/s, Yaw: {angular_vel:.2f} rad/s")
            
        time.sleep(0.02)

def send_command_loop():
    global robot_ip, is_running, start_time
    send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    # Wait a bit for prints to settle
    time.sleep(0.5)
    print_command_menu()

    while True:
        try:
            cmd = input().strip()
            if not cmd:
                continue
            
            with state_lock:
                target_ip = robot_ip
                
            if target_ip is None:
                print("[WARN] Robot IP is currently unknown. Wait until telemetry starts streaming.")
                continue

            # Command formatting
            if cmd == 'M':
                manual_control_mode(send_sock, target_ip)
                print_command_menu()
                continue
            elif cmd == 'I':
                with state_lock:
                    is_running = True
                    start_time = time.time()
                payload = "C,I"
            elif cmd == 'S':
                with state_lock:
                    is_running = False
                    start_time = None
                payload = "C,S"
            elif cmd in ['R', 'C']:
                payload = f"C,{cmd}"
            elif cmd.startswith('V,') or cmd.startswith('K,'):
                payload = cmd
            else:
                print("[ERROR] Unknown format. Use I, R, C, S, M, K,left,right or V,linear,angular (e.g. V,0.2,-0.5)")
                continue
                
            send_sock.sendto(payload.encode("utf-8"), (target_ip, UDP_SEND_PORT))
            print(f"[SENT] Command '{payload}' sent to robot at {target_ip}:{UDP_SEND_PORT}")
            
        except KeyboardInterrupt:
            print("\n[INFO] Shutting down command interface.")
            break
        except Exception as e:
            print(f"[ERROR] Failed to send command: {e}")

if __name__ == "__main__":
    # Start receiver thread
    t = threading.Thread(target=listen_telemetry, daemon=True)
    t.start()
    
    # Run user input loop in the main thread
    try:
        send_command_loop()
    except KeyboardInterrupt:
        print("\nExiting.")
