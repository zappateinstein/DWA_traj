#include "dwa2Dtrajectory_impl.h"
#include "dwa2Dtrajectory.h"
#include <Matrix.h>
#include <Layout.h>
#include <GroupBox.h>
#include <DoubleSpinBox.h>
#include <cmath>
#include <limits>
#include <iostream>

using std::string;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::filter;

dwa2Dtrajectory_impl::dwa2Dtrajectory_impl(
    dwa2Dtrajectory *self, const LayoutPosition *position, 
    string name1) {
    
    first_update = true;
    is_running = false;
    is_finishing = false;
    have_end = false;

    // ========== UI : Paramètres de mouvement ==========
    GroupBox *reglages_groupbox = new GroupBox(position, name1);
    T = new DoubleSpinBox(reglages_groupbox->NewRow(), "period, 0 for auto", " s", 0.0, 1.0, 0.01);
    velocity = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(), "v_max", " m/s", 0.0, 5.0, 0.1, 1.0);
    angular = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(), "w_max", " rad/s", 0.0, 3.14, 0.1, 1.0);
    
    // ========== UI : Paramètres DWA ==========
    //GroupBox *dwa_reglages = new GroupBox(position, "DWA Parameters");
    alpha = new DoubleSpinBox(reglages_groupbox->NewRow(), "alpha (heading)", "", 0.0, 10.0, 0.1, 1.5);
    beta = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(), "beta (velocity)", "", 0.0, 10.0, 0.1, 1.0);
    gamma = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(), "gamma (obstacle)", "", 0.0, 10.0, 0.1, 1.0);
    epsilon = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(), "epsilon (collision)", " m", 0.0, 1.0, 0.01, 2, 0.1);
    finalpositionx = new DoubleSpinBox (reglages_groupbox->NewRow(), "final position x", " m", -5.0, 5.0, 0.1);
    finalpositiony = new DoubleSpinBox (reglages_groupbox->LastRowLastCol(), "final position y", " m", -5.0, 5.0, 0.1);
    nb_obstacles = new DoubleSpinBox (reglages_groupbox->LastRowLastCol(), "number of obstacles", "", 0, 30, 1, 1);
    
    // ========== Matrice de sortie (pos, vel, acc, jerk) ==========
    MatrixDescriptor *desc = new MatrixDescriptor(4, 2);
    desc->SetElementName(0, 0, "pos.x");
    desc->SetElementName(0, 1, "pos.y");
    desc->SetElementName(1, 0, "vel.x");
    desc->SetElementName(1, 1, "vel.y");
    desc->SetElementName(2, 0, "acc.x");
    desc->SetElementName(2, 1, "acc.y");
    desc->SetElementName(3, 0, "jerk.x");
    desc->SetElementName(3, 1, "jerk.y");
    output = new Matrix(self, desc, floatType, name1);
    delete desc;

    // ========== Initialisation de l'état ==========
    pos = Vector2Df(0, 0);
    vel = Vector2Df(0, 0);
    w_current = 0.0f;
    acc = Vector2Df(0, 0);
    jerk = Vector2Df(0, 0);
    end_speed = Vector2Df(0, 0);
    goal_pos = Vector2Df(0, 0);
    goal_pos.x = finalpositionx ->Value();
    goal_pos.y = finalpositiony ->Value();
    pos_off = Vector2Df(0, 0);    // pas d'offset par défaut
    vel_off = Vector2Df(0, 0);
    angle_off = 0.0f;
    CurrentTime = 0.0f;

    // Paramètres DWA par défaut
    params.v_max = 1.0f;
    params.w_max = 1.0f;
    params.dt = 0.1f;
    params.T = 0.9f;
    params.alpha = 1.5f;
    params.beta = 1.0f;
    params.gamma = 1.0f;
    params.epsilon = 0.1f;
    params.dv = 0.02f;
    params.dw = 0.02f;
    params.sim_time = 30.0f;

    std::cerr << "[DWA_impl] Initialized successfully\n";
}

dwa2Dtrajectory_impl::~dwa2Dtrajectory_impl() {
    delete output;
}

// ========== GESTION DES OBSTACLES ==========

void dwa2Dtrajectory_impl::SetObstacles(const std::vector<Obstacle> &obs) {
    obstacles = obs;
    std::cerr << "[DWA] Set " << obstacles.size() << " obstacles\n";
}

void dwa2Dtrajectory_impl::AddObstacle(float x, float y, float radius) {
    obstacles.push_back(Obstacle(x, y, radius));
    std::cerr << "[DWA] Added obstacle at (" << x << ", " << y << ") r=" << radius << "\n";
}

void dwa2Dtrajectory_impl::ClearObstacles() {
    obstacles.clear();
    std::cerr << "[DWA] Cleared all obstacles\n";
}

// ========== GETTERS/SETTERS ==========

void dwa2Dtrajectory_impl::SetEnd(const Vector2Df &e) {
    goal_pos = e;  // Définit la position cible (goal)
    have_end = true;
    std::cerr << "[DWA] Goal set to (" << e.x << ", " << e.y << ")\n";
}

void dwa2Dtrajectory_impl::SetEndSpeed(const Vector2Df &vs) {
    end_speed = vs;
}

void dwa2Dtrajectory_impl::GetEnd(Vector2Df &p) const {
    p = goal_pos;  // Retourne la position cible
}

void dwa2Dtrajectory_impl::GetPos(Vector2Df &p) const { 
    p = pos + pos_off;  // Position avec offset appliqué
}

void dwa2Dtrajectory_impl::GetVel(Vector2Df &v) const { 
    v = vel + vel_off;  // Vitesse avec offset appliqué
}

void dwa2Dtrajectory_impl::GetAcc(Vector2Df &a) const { 
    a = acc; 
}

void dwa2Dtrajectory_impl::GetJerk(Vector2Df &j) const { 
    j = jerk; 
}

void dwa2Dtrajectory_impl::GetCurrentTime(float &t) const {
    t = CurrentTime;
}

// ========== CONTRÔLE DE TRAJECTOIRE ==========

void dwa2Dtrajectory_impl::StartTraj(const Vector2Df &start_pos) {
    if (!have_end) {
        std::cerr << "[DWA] ERROR: No goal set, cannot start!\n";
        return;
    }
    
    is_running = true;
    is_finishing = false;
    first_update = true;
    
    pos = start_pos;
    vel = Vector2Df(0, 0);
    acc = Vector2Df(0, 0);
    jerk = Vector2Df(0, 0);
    
    // Orientation initiale vers le goal
    Vector2Df dir = goal_pos - pos;
    angle_off = atan2f(dir.y, dir.x);
    
    CurrentTime = 0.0f;
    
    std::cerr << "[DWA] Started from (" << pos.x << ", " << pos.y 
              << ") toward goal (" << goal_pos.x << ", " << goal_pos.y << ")\n";
}

void dwa2Dtrajectory_impl::SetCurrentPosition(const Vector2Df &current_pos, float current_yaw) {
    pos = current_pos;
    angle_off = current_yaw;
}

void dwa2Dtrajectory_impl::FinishTraj(void) {
    is_finishing = true;
    std::cerr << "[DWA] Finishing trajectory\n";
}

void dwa2Dtrajectory_impl::StopTraj(void) {
    is_running = false;
    std::cerr << "[DWA] Trajectory stopped\n";
}

// ========== FONCTIONS MATHÉMATIQUES ==========

float dwa2Dtrajectory_impl::AngDiff(float a, float b) {
    float diff = fmodf(a - b + M_PI, 2.0f * M_PI);
    if (diff < 0.0f) diff += 2.0f * M_PI;
    return diff - M_PI;
}

void dwa2Dtrajectory_impl::RobotMotion(float &px, float &py, float &theta, 
                                        float v, float w, float dt) {
    px += v * cosf(theta) * dt;
    py += v * sinf(theta) * dt;
    theta += w * dt;
}

// ========== SIMULATION DE TRAJECTOIRE ==========

SimulatedTrajectory dwa2Dtrajectory_impl::SimTrajectory(float v, float w, 
                                                          float dt, float T) {
    int steps = std::max(1, static_cast<int>(T / dt));
    SimulatedTrajectory traj;
    traj.xs.resize(steps);
    traj.ys.resize(steps);
    traj.thetas.resize(steps);
    
    float px = pos.x;
    float py = pos.y;
    float theta = angle_off;
    
    for (int i = 0; i < steps; ++i) {
        RobotMotion(px, py, theta, v, w, dt);
        traj.xs[i] = px;
        traj.ys[i] = py;
        traj.thetas[i] = theta;
    }
    
    return traj;
}

// ========== DISTANCE MINIMALE AUX OBSTACLES ==========

float dwa2Dtrajectory_impl::MinimalDistance(const SimulatedTrajectory &traj) {
    if (obstacles.empty()) {
        return std::numeric_limits<float>::infinity();
    }
    
    float min_dist = std::numeric_limits<float>::infinity();
    
    for (size_t i = 0; i < traj.xs.size(); ++i) {
        float px = traj.xs[i];
        float py = traj.ys[i];
        
        for (const auto &obs : obstacles) {
            float dx = px - obs.x;
            float dy = py - obs.y;
            float dist = sqrtf(dx * dx + dy * dy) - (obs.radius+0.15f);
            
            if (dist < min_dist) {
                min_dist = dist;
            }
        }
    }
    
    return min_dist;
}

// ========== ÉVALUATION DE TRAJECTOIRE ==========

float dwa2Dtrajectory_impl::EvaluateTrajectory(const SimulatedTrajectory &traj, 
                                                 float v) {
    if (traj.xs.empty()) return -std::numeric_limits<float>::infinity();
    
    // Point final de la trajectoire
    float x_end = traj.xs.back();
    float y_end = traj.ys.back();
    float theta_end = traj.thetas.back();
    
    // 1. Heading score: orientation vers le goal
    float goal_dir = atan2f(goal_pos.y - y_end, goal_pos.x - x_end);
    float heading = 1.0f - fabsf(AngDiff(goal_dir, theta_end)) / M_PI;
    
    // 2. Velocity score: favorise les vitesses élevées
    float velocity_term = v / params.v_max;
    
    // 3. Obstacle score: distance minimale aux obstacles
    float d_min = MinimalDistance(traj);
    
    // Collision check
    if (d_min < params.epsilon) {
        return -std::numeric_limits<float>::infinity();
    }
    
    // Score final pondéré
    float score = params.alpha * heading + 
                  params.beta * velocity_term + 
                  params.gamma * d_min;
    
    return score;
}

// ========== CALCUL DE LA COMMANDE DWA ==========

void dwa2Dtrajectory_impl::CalcVelocityCommand(float &v_cmd, float &w_cmd) {
    float best_score = -std::numeric_limits<float>::infinity();
    v_cmd = 0.0f;
    w_cmd = 0.0f;
    
    // ========== FENÊTRE DYNAMIQUE ==========
    // Vitesse linéaire actuelle
    float v_current = sqrtf(vel.x * vel.x + vel.y * vel.y);
    
    // Accélérations max (valeurs fixes puisque pas de widgets UI)
    float acc_max = 1.0f;      // m/s²
    float acc_w_max = 3.14f;    // rad/s²
    
    // Fenêtre dynamique pour v (vitesses atteignables)
    float v_min = std::max(0.0f, v_current - acc_max * params.dt);
    float v_max_dyn = std::min(params.v_max, v_current + acc_max * params.dt);
    
    // Fenêtre dynamique pour w
    float w_min = std::max(-params.w_max, w_current - acc_w_max * params.dt);
    float w_max_dyn = std::min(params.w_max, w_current + acc_w_max * params.dt);
    
    std::cerr << "[DWA] Dynamic window: v=[" << v_min << ", " << v_max_dyn 
              << "] w=[" << w_min << ", " << w_max_dyn << "]\n";
    
    // Exploration dans la fenêtre dynamique
    for (float v = v_min; v <= v_max_dyn; v += params.dv) {
        for (float w = w_min; w <= w_max_dyn; w += params.dw) {
            
            SimulatedTrajectory traj = SimTrajectory(v, w, params.dt, params.T);
            
            float score = EvaluateTrajectory(traj, v);
            
            // Prefer higher velocity if scores are very close
            if (score > best_score) {
                best_score = score;
                v_cmd = v;
                w_cmd = w;
            }
        }
    }
    
    // Debug: affiche la commande choisie
    std::cerr << "[DWA] Best: v=" << v_cmd << " w=" << w_cmd 
              << " score=" << best_score << "\n";
}

// ========== UPDATE PRINCIPAL ==========

void dwa2Dtrajectory_impl::Update(Time time) {
    // ========== Calcul du delta_t ==========
    float delta_t;
    if (T->Value() == 0.0f) {
        if (first_update) {
            first_update = false;
            previous_time = time;
            output->SetDataTime(time);
            return;
        }
        delta_t = static_cast<float>(time - previous_time) / 1000000000.0f;
    } else {
        delta_t = T->Value();
    }
    previous_time = time;
    
    // ========== Si non actif, publie l'état courant ==========
    if (!is_running) {
        output->GetMutex();
        output->SetValueNoMutex(0, 0, pos.x);
        output->SetValueNoMutex(0, 1, pos.y);
        output->SetValueNoMutex(1, 0, vel.x);
        output->SetValueNoMutex(1, 1, vel.y);
        output->SetValueNoMutex(2, 0, acc.x);
        output->SetValueNoMutex(2, 1, acc.y);
        output->SetValueNoMutex(3, 0, jerk.x);
        output->SetValueNoMutex(3, 1, jerk.y);
        output->ReleaseMutex();
        output->SetDataTime(time);
        return;
    }
    
    // ========== Mise à jour des paramètres UI ==========
    params.v_max = velocity->Value();
    params.w_max = angular->Value();
    std::cerr << "[DWA] vitesse maximale=" << params.v_max <<"vitesse angulaire maximale"<<params.w_max<< "\n";
    params.alpha = alpha->Value();
    params.beta = beta->Value();
    params.gamma = gamma->Value();
    params.epsilon = epsilon->Value();
    params.dt = delta_t;
    
    // ========== Vérification d'arrivée au goal ==========
    Vector2Df goal_vec = goal_pos - pos;
    float dist_to_goal = goal_vec.GetNorm();
    
    if (dist_to_goal < 0.1f || is_finishing) {
        // Arrêt progressif
        vel = vel * 0.9f;  // décélération
        is_running = false;
        std::cerr << "[DWA] Goal reached! Distance=" << dist_to_goal << "\n";
        std::cerr << "[DWA] Mission terminée ! tsim = " << CurrentTime << " s\n";
    } else {
        // ========== Calcul de la commande simple (contrôleur proportionnel) ==========
        // CORRECTION: Calcul de l'angle vers le goal depuis la position ACTUELLE
       float v_cmd, w_cmd;
       CalcVelocityCommand(v_cmd, w_cmd);
        std::cerr << "[DWA] v_cmd=" << v_cmd << " w_cmd=" << w_cmd 
                  << " dist_to_goal=" << dist_to_goal << "\n";
        
        // ========== Mise à jour de l'état du robot ==========
        // La position (pos) est déjà mise à jour par SetCurrentPosition() appelé depuis dwatraj.cpp
        // On calcule juste les commandes et les dérivées
        
        float old_px = pos.x;
        float old_py = pos.y;
        float old_vx = vel.x;
        float old_vy = vel.y;   
        float old_ax = acc.x;
        float old_ay = acc.y;
        float old_angle = angle_off;
        
        // Mise à jour de la vitesse (dérivée de position)
        // pos a été mis à jour par SetCurrentPosition(), on calcule vel depuis le changement
        vel.x = (pos.x - old_px) / delta_t;
        vel.y = (pos.y - old_py) / delta_t;
        
        // Mise à jour de la vitesse angulaire actuelle
        w_current = (angle_off - old_angle) / delta_t;
        
        // Mise à jour de l'accélération (dérivée de vitesse)
        acc.x = (vel.x - old_vx) / delta_t;
        acc.y = (vel.y - old_vy) / delta_t;
        
        // Mise à jour du jerk (dérivée de l'accélération)
        jerk.x = (acc.x - old_ax) / delta_t;
        jerk.y = (acc.y - old_ay) / delta_t;
    }
    
    CurrentTime += delta_t;
    
    // ========== Publication des sorties ==========
    output->GetMutex();
    output->SetValueNoMutex(0, 0, pos.x);  // Position avec offset
    output->SetValueNoMutex(0, 1, pos.y);
    output->SetValueNoMutex(1, 0, vel.x);  // Vitesse avec offset
    output->SetValueNoMutex(1, 1, vel.y);
    output->SetValueNoMutex(2, 0, acc.x);
    output->SetValueNoMutex(2, 1, acc.y);
    output->SetValueNoMutex(3, 0, jerk.x);
    output->SetValueNoMutex(3, 1, jerk.y);
    output->ReleaseMutex();
    output->SetDataTime(time);
}