import socket
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation

# --- CONFIGURATION RESEAU ---
UDP_IP = "127.0.0.1"
UDP_PORT = 9005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)

print(f"En écoute sur {UDP_IP}:{UDP_PORT}...")

# --- MISE EN PLACE DU GRAPHIQUE ---
fig, ax = plt.subplots(figsize=(8, 8))
ax.set_xlim(-6, 6)
ax.set_ylim(-6, 6)
ax.grid(True)
ax.set_title("Visualisation DWA en Temps Réel")

# Création des objets graphiques vides
robot_dot, = ax.plot([], [], 'ro', markersize=8, label='Robot')
goal_dot, = ax.plot([], [], 'gx', markersize=10, markeredgewidth=3, label='But')
path_traj, = ax.plot([], [], 'b-', alpha=0.5, label='Trajectoire')

# Mémoire
history_x = []
history_y = []
obstacles_list = []

def update(frame):
    # 1. On récupère la DERNIÈRE donnée (on vide le buffer)
    try:
        data = None
        while True:
            try:
                # On lit tant qu'il y a des paquets
                packet, _ = sock.recvfrom(4096)
                data = packet
            except BlockingIOError:
                # Plus rien à lire, on sort de la boucle
                break
        
        # 2. Si on n'a rien reçu du tout dans ce tour, on ne fait rien
        if data is None:
            return robot_dot, goal_dot, path_traj

        # 3. Traitement du dernier message reçu
        msg = data.decode('utf-8')
        parts = msg.split(',')

        if parts[0] == "DWA":
            # --- ROBOT ---
            rx = float(parts[2])
            ry = float(parts[3])
            robot_dot.set_data([rx], [ry])
            
            # Trace
            history_x.append(rx)
            history_y.append(ry)
            path_traj.set_data(history_x, history_y)

            # --- BUT / GOAL ---
            gx = float(parts[7])
            gy = float(parts[8])
            goal_dot.set_data([gx], [gy])

            # --- OBSTACLES ---
            global obstacles_list
            # Effacer les anciens
            for obs in obstacles_list:
                obs.remove()
            obstacles_list = []

            # Recréer les nouveaux
            nb_obs = int(parts[9])
            for i in range(nb_obs):
                idx = 10 + (i * 3)
                ox = float(parts[idx])
                oy = float(parts[idx + 1])
                r  = float(parts[idx + 2])
                
                # Création cercle (gris avec bord noir)
                c = patches.Circle((ox, oy), r, fc='gray', ec='black', alpha=0.5)
                ax.add_patch(c)
                obstacles_list.append(c)

    except Exception as e:
        print(f"Erreur: {e}")

    return robot_dot, goal_dot, path_traj

# --- LANCEMENT DE L'ANIMATION ---
# interval=50 veut dire qu'on rafraîchit l'image toutes les 50ms
ani = FuncAnimation(fig, update, interval=50, blit=False)

# Affichage final
plt.show()