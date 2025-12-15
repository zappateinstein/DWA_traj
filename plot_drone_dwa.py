import socket
import cv2
import numpy as np
import time
from collections import deque
import signal
import sys
import math

# Configuration
UDP_IP = "0.0.0.0"
UDP_PORT = 9005
BUFFER_SIZE = 65535

# Map Configuration
MAP_SIZE = 800  # pixels
MAP_SCALE = 50  # pixels per meter
MAP_CENTER_X = MAP_SIZE // 2
MAP_CENTER_Y = MAP_SIZE // 2

# Data storage
ugv_path = deque(maxlen=1000)
ugv_pos = (0, 0)
ugv_yaw = 0.0
target_pos = (0, 0)
obstacles = []

# Setup UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)

print(f"Listening on {UDP_IP}:{UDP_PORT}")

def signal_handler(sig, frame):
    print('Exiting...')
    sock.close()
    cv2.destroyAllWindows()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

def world_to_map(x, y):
    # World X -> Map X (right)
    # World Y -> Map Y (up) - so we invert Y for screen coords
    px = int(MAP_CENTER_X + x * MAP_SCALE)
    py = int(MAP_CENTER_Y - y * MAP_SCALE)
    return (px, py)

def draw_map():
    # Create white background
    map_img = np.ones((MAP_SIZE, MAP_SIZE, 3), dtype=np.uint8) * 255

    # Draw grid
    grid_color = (220, 220, 220)
    axis_color = (150, 150, 150)
    
    # Grid lines every 1 meter
    for i in range(-10, 11):
        p1 = world_to_map(i, -10)
        p2 = world_to_map(i, 10)
        cv2.line(map_img, p1, p2, grid_color, 1)
        
        p1 = world_to_map(-10, i)
        p2 = world_to_map(10, i)
        cv2.line(map_img, p1, p2, grid_color, 1)

    # Axes
    origin = world_to_map(0, 0)
    cv2.line(map_img, (0, origin[1]), (MAP_SIZE, origin[1]), axis_color, 2) # X axis
    cv2.line(map_img, (origin[0], 0), (origin[0], MAP_SIZE), axis_color, 2) # Y axis

    # Draw Obstacles
    for obs in obstacles:
        ox, oy, orad = obs
        pt = world_to_map(ox, oy)
        rad_px = int(orad * MAP_SCALE)
        cv2.circle(map_img, pt, rad_px, (0, 0, 0), -1) # Black filled circle

    # Draw UGV path
    if len(ugv_path) > 1:
        pts = [world_to_map(p[0], p[1]) for p in ugv_path]
        cv2.polylines(map_img, [np.array(pts)], False, (255, 0, 0), 2)

    # Draw Target
    target_pt = world_to_map(target_pos[0], target_pos[1])
    cv2.drawMarker(map_img, target_pt, (0, 0, 255), cv2.MARKER_CROSS, 15, 3)
    cv2.putText(map_img, "Target", (target_pt[0]+10, target_pt[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    # Draw UGV
    ugv_pt = world_to_map(ugv_pos[0], ugv_pos[1])
    
    # Draw UGV orientation
    arrow_len = 20
    end_x = int(ugv_pt[0] + arrow_len * math.cos(ugv_yaw))
    end_y = int(ugv_pt[1] - arrow_len * math.sin(ugv_yaw)) # Y inverted
    cv2.arrowedLine(map_img, ugv_pt, (end_x, end_y), (255, 0, 0), 2, tipLength=0.3)
    cv2.circle(map_img, ugv_pt, 6, (255, 0, 0), -1)
    
    cv2.putText(map_img, f"Pos: ({ugv_pos[0]:.2f}, {ugv_pos[1]:.2f})", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

    cv2.imshow("DWA Trajectory", map_img)

def main():
    global ugv_pos, ugv_yaw, target_pos, obstacles

    cv2.namedWindow("DWA Trajectory")

    while True:
        # Process all pending UDP packets
        while True:
            try:
                data, addr = sock.recvfrom(BUFFER_SIZE)
                text = data.decode('utf-8')
                
                parts = text.split(',')
                # DWA,timestamp,ugv_x,ugv_y,ugv_yaw,ugv_vx,ugv_vy,target_x,target_y,obs_count,[obs_x,obs_y,obs_r...]
                
                if parts[0] == "DWA" and len(parts) >= 10:
                    ugv_pos = (float(parts[2]), float(parts[3]))
                    ugv_yaw = float(parts[4])
                    target_pos = (float(parts[7]), float(parts[8]))
                    
                    ugv_path.append(ugv_pos)
                    
                    obs_count = int(parts[9])
                    
                    # Parse obstacles
                    if obs_count > 0:
                        new_obstacles = []
                        idx = 10
                        for _ in range(obs_count):
                             if idx + 2 >= len(parts): break
                             ox = float(parts[idx])
                             oy = float(parts[idx+1])
                             orad = float(parts[idx+2])
                             new_obstacles.append((ox, oy, orad))
                             idx += 3
                        obstacles = new_obstacles

            except BlockingIOError:
                break
            except Exception as e:
                print(f"Packet error: {e}")
                break

        draw_map()
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

if __name__ == "__main__":
    main()
