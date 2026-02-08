import socket
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import numpy as np

# ============================================================================
# CONFIGURATION RÉSEAU
# ============================================================================
UDP_IP = "127.0.0.1"
UDP_PORT = 9006

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)

print(f"[DWA Plot] Listening on {UDP_IP}:{UDP_PORT}")

# ============================================================================
# CONFIGURATION GRAPHIQUES
# ============================================================================
# Figure 1: Carte de navigation en temps réel
fig_map = plt.figure(1, figsize=(8, 8))
ax_map = fig_map.add_subplot(111)
ax_map.set_xlim(-6, 6)
ax_map.set_ylim(-6, 6)
ax_map.set_xlabel('X (m)')
ax_map.set_ylabel('Y (m)')
ax_map.set_title('DWA Real-Time Navigation Map')
ax_map.grid(True)

robot_dot, = ax_map.plot([], [], 'ro', markersize=10, label='Robot')
goal_dot, = ax_map.plot([], [], 'gx', markersize=15, markeredgewidth=3, label='Goal')
path_traj, = ax_map.plot([], [], 'b-', alpha=0.6, linewidth=2, label='Trajectory')
ax_map.legend(loc='upper right')

# ============================================================================
# VARIABLES GLOBALES
# ============================================================================
history = {
    't': [],
    'x': [],
    'y': [],
    'v': [],
    'w': [],
    'yaw': []
}
obstacles_list = []
t_start = None
mission_ended = False
mission_end_signal_time = None
post_mission_frames = 0

# ============================================================================
# FONCTION D'ANIMATION (APPELÉE À CHAQUE FRAME)
# ============================================================================
def update_map(frame):
    global t_start, mission_ended, obstacles_list, mission_end_signal_time, post_mission_frames
    
    # Vidage du buffer UDP pour obtenir le dernier message
    data = None
    try:
        while True:
            try:
                packet, _ = sock.recvfrom(4096)
                data = packet
            except BlockingIOError:
                break
    except Exception as e:
        print(f"[Error] Socket read: {e}")
        return robot_dot, goal_dot, path_traj
    
    # Si pas de données, on retourne
    if data is None:
        return robot_dot, goal_dot, path_traj
    
    # Si mission déjà terminée, ne plus traiter les données
    if mission_ended:
        return robot_dot, goal_dot, path_traj

    # Décodage du message
    try:
        msg = data.decode('utf-8')
        parts = msg.split(',')

        if parts[0] != "DWA":
            return robot_dot, goal_dot, path_traj

        # --- EXTRACTION DES DONNÉES ---
        t_abs = float(parts[1])
        if t_start is None:
            t_start = t_abs
            print(f"[DWA Plot] Mission started - recording data...")
            print(f"[DWA Plot] DEBUG - First timestamp: {t_abs}")
        
        t = t_abs - t_start
        
        # Détecter si le temps fait un GROS saut en arrière (nouveau départ)
        # Cela indique que start_traj a été cliqué et mission_time réinitialisé
        if len(history['t']) > 0 and (history['t'][-1] - t) > 0.5:
            print(f"[DWA Plot] ========================================")
            print(f"[DWA Plot] Mission restart detected!")
            print(f"[DWA Plot] Time jumped from {history['t'][-1]:.2f}s back to {t:.2f}s")
            print(f"[DWA Plot] Resetting for new mission...")
            print(f"[DWA Plot] ========================================")
            
            # Réinitialiser tout pour la nouvelle mission
            t_start = t_abs
            t = 0.0
            history['t'].clear()
            history['x'].clear()
            history['y'].clear()
            history['v'].clear()
            history['w'].clear()
            history['yaw'].clear()
            mission_ended = False
            mission_end_signal_time = None
            post_mission_frames = 0
        
        rx, ry = float(parts[2]), float(parts[3])
        yaw = float(parts[4])
        vx, vy = float(parts[5]), float(parts[6])
        gx, gy = float(parts[7]), float(parts[8])
        
        # NOUVEAU: Lire le flag mission_ended depuis le C++
        mission_ended_flag = int(parts[9])
        
        # --- CALCUL DES VITESSES ---
        v_mag = np.sqrt(vx**2 + vy**2)
        
        w_val = 0.0
        if len(history['t']) > 0:
            dt = t - history['t'][-1]
            if dt > 0:
                diff = yaw - history['yaw'][-1]
                diff = (diff + np.pi) % (2 * np.pi) - np.pi
                w_val = diff / dt

        # --- ENREGISTREMENT DES DONNÉES ---
        history['t'].append(t)
        history['x'].append(rx)
        history['y'].append(ry)
        history['v'].append(v_mag)
        history['w'].append(w_val)
        history['yaw'].append(yaw)

        # --- CALCUL DISTANCE AU BUT ---
        dist_to_goal = np.sqrt((gx - rx)**2 + (gy - ry)**2)
        
        # --- DÉTECTION DE FIN DE MISSION DEPUIS C++ ---
        if mission_ended_flag == 1 and not mission_ended:
            mission_ended = True
            print(f"[DWA Plot] ========================================")
            print(f"[DWA Plot] Mission end signal received from C++!")
            print(f"[DWA Plot] Data points recorded: {len(history['t'])}")
            print(f"[DWA Plot] Mission duration: {t:.2f}s")
            print(f"[DWA Plot] Final position: ({rx:.2f}, {ry:.2f})")
            print(f"[DWA Plot] Goal position: ({gx:.2f}, {gy:.2f})")
            print(f"[DWA Plot] Distance to goal: {dist_to_goal:.3f}m")
            print(f"[DWA Plot] ========================================")
            
            if len(history['t']) > 10:
                plot_velocities()
            else:
                print("[DWA Plot] Not enough data for velocity plots")
            
            return robot_dot, goal_dot, path_traj

        # --- MISE À JOUR DE LA CARTE ---
        robot_dot.set_data([rx], [ry])
        goal_dot.set_data([gx], [gy])
        path_traj.set_data(history['x'], history['y'])

        # --- MISE À JOUR DES OBSTACLES ---
        for obs in obstacles_list:
            obs.remove()
        obstacles_list = []
        
        # MODIFIÉ: L'index du nombre d'obstacles est maintenant 10 (après mission_ended_flag)
        nb_obs = int(parts[10])
        for i in range(nb_obs):
            idx = 11 + (i * 3)  # MODIFIÉ: Commence à 11 au lieu de 10
            o_x, o_y, o_r = float(parts[idx]), float(parts[idx+1]), float(parts[idx+2])
            c = patches.Circle((o_x, o_y), o_r, fc='gray', ec='black', alpha=0.5)
            ax_map.add_patch(c)
            obstacles_list.append(c)

    except Exception as e:
        print(f"[Error] Processing data: {e}")
        import traceback
        traceback.print_exc()

    return robot_dot, goal_dot, path_traj

# ============================================================================
# FONCTION D'AFFICHAGE DES VITESSES (APRÈS ARRÊT)
# ============================================================================
def plot_velocities():
    """Génère les graphiques de vitesse après la fin de la mission"""
    if len(history['t']) < 2:
        print("[DWA Plot] Not enough data for velocity plots")
        return
    
    print("[DWA Plot] Generating velocity plots...")
    print(f"[DWA Plot] Time range: {min(history['t']):.2f}s to {max(history['t']):.2f}s")
    print(f"[DWA Plot] Velocity range: {min(history['v']):.3f} to {max(history['v']):.3f} m/s")
    
    # Nouvelle figure pour les vitesses
    fig_vel = plt.figure(2, figsize=(14, 7))
    fig_vel.suptitle(f'Mission Velocities (Duration: {max(history["t"]):.2f}s)', fontsize=14)
    
    # Sous-graphique 1: Vitesse linéaire
    ax_v = fig_vel.add_subplot(211)
    ax_v.plot(history['t'], history['v'], 'g-', linewidth=2, marker='o', markersize=2)
    ax_v.set_ylabel('Linear velocity (m/s)', fontsize=12)
    ax_v.set_title('Linear Velocity over Time', fontsize=12, fontweight='bold')
    ax_v.grid(True, alpha=0.3)
    
    # Forcer les limites correctes
    t_min, t_max = min(history['t']), max(history['t'])
    v_min, v_max = min(history['v']), max(history['v'])
    
    print(f"[DWA Plot] DEBUG - t_min={t_min:.4f}, t_max={t_max:.4f}")
    print(f"[DWA Plot] DEBUG - v_min={v_min:.4f}, v_max={v_max:.4f}")
    print(f"[DWA Plot] DEBUG - Number of points: {len(history['t'])}")
    
    # Forcer xlim avec marge
    if t_max - t_min < 0.1:
        # Si durée très courte, afficher au moins 1 seconde
        ax_v.set_xlim(-0.1, 1.0)
    else:
        ax_v.set_xlim(t_min - 0.5, t_max + 0.5)
    
    ax_v.set_ylim(max(0, v_min - 0.1), v_max + 0.2)
    
    # Sous-graphique 2: Vitesse angulaire
    ax_w = fig_vel.add_subplot(212)
    ax_w.plot(history['t'], history['w'], 'm-', linewidth=2, marker='o', markersize=2)
    ax_w.set_xlabel('Time (s)', fontsize=12)
    ax_w.set_ylabel('Angular velocity (rad/s)', fontsize=12)
    ax_w.set_title('Angular Velocity over Time', fontsize=12, fontweight='bold')
    ax_w.grid(True, alpha=0.3)
    
    w_min, w_max = min(history['w']), max(history['w'])
    w_range = max(abs(w_min), abs(w_max))
    
    # Même xlim que le graphe du dessus
    if t_max - t_min < 0.1:
        ax_w.set_xlim(-0.1, 1.0)
    else:
        ax_w.set_xlim(t_min - 0.5, t_max + 0.5)
    
    ax_w.set_ylim(-w_range - 0.1, w_range + 0.1)
    
    fig_vel.tight_layout()
    fig_vel.canvas.draw()
    fig_vel.show()
    
    # Statistiques détaillées
    print(f"[DWA Plot] ========== STATISTICS ==========")
    print(f"[DWA Plot] Data points: {len(history['t'])}")
    print(f"[DWA Plot] Max linear velocity: {v_max:.3f} m/s")
    print(f"[DWA Plot] Avg linear velocity: {np.mean(history['v']):.3f} m/s")
    if history['w']:
        print(f"[DWA Plot] Max angular velocity: {max(abs(w) for w in history['w']):.3f} rad/s")
    
    # NOUVEAU: Sauvegarder les données dans un CSV pour analyse ultérieure
    try:
        import csv
        filename = f"dwa_mission_{int(t_start)}.csv"
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['time', 'x', 'y', 'v', 'w', 'yaw'])
            for i in range(len(history['t'])):
                writer.writerow([
                    history['t'][i],
                    history['x'][i],
                    history['y'][i],
                    history['v'][i],
                    history['w'][i],
                    history['yaw'][i]
                ])
        print(f"[DWA Plot] Data saved to {filename}")
    except Exception as e:
        print(f"[DWA Plot] Could not save CSV: {e}")

# ============================================================================
# LANCEMENT DE L'ANIMATION
# ============================================================================
ani = FuncAnimation(fig_map, update_map, interval=50, blit=False, cache_frame_data=False)
plt.tight_layout()
plt.show()