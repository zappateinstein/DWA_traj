#ifndef DWA2DTRAJECTORY_IMPL_H
#define DWA2DTRAJECTORY_IMPL_H

#include <Object.h>
#include <Vector2D.h>
#include <vector>
#include <cstddef>

namespace flair {
namespace core {
class Matrix;
class io_data;
}
namespace gui {
class LayoutPosition;
class DoubleSpinBox;
}
namespace filter {
class dwa2Dtrajectory;
}
}

// Structure pour représenter un obstacle ponctuel
struct Obstacle {
    float x;
    float y;
    float radius;  // rayon de sécurité autour de l'obstacle
    
    Obstacle(float x_, float y_, float r_ = 0.3f) : x(x_), y(y_), radius(r_) {}
};

// Structure pour une trajectoire simulée
struct SimulatedTrajectory {
    std::vector<float> xs;
    std::vector<float> ys;
    std::vector<float> thetas;
};

class dwa2Dtrajectory_impl {
public:
    dwa2Dtrajectory_impl(flair::filter::dwa2Dtrajectory *self,
                         const flair::gui::LayoutPosition *position,
                         std::string name1);
    ~dwa2Dtrajectory_impl();

    void Update(flair::core::Time time);
    void StartTraj(const flair::core::Vector2Df &start_pos);
    void FinishTraj(void);
    void StopTraj(void);

    // Gestion des obstacles
    void SetObstacles(const std::vector<Obstacle> &obs);
    void AddObstacle(float x, float y, float radius = 0.3f);
    void ClearObstacles();

    // offsets applied to outputs
    flair::core::Vector2Df pos_off;   // OFFSET appliqué à la sortie (repère)
    flair::core::Vector2Df vel_off;   // OFFSET appliqué à la vitesse
    flair::core::Vector2Df goal_pos;  // position de l'objectif
    
    // État du robot
    float angle_off;                   // orientation actuelle (theta)

    bool is_running;
    flair::core::Matrix *output;

    // getters et setters pour position, vitesse, acc, jerk
    void GetPos(flair::core::Vector2Df &p) const;
    void GetVel(flair::core::Vector2Df &v) const;
    void GetAcc(flair::core::Vector2Df &a) const;
    void GetJerk(flair::core::Vector2Df &j) const;

    void SetEnd(const flair::core::Vector2Df &e);
    void SetEndSpeed(const flair::core::Vector2Df &vs);
    void GetEnd(flair::core::Vector2Df &p) const;

    // Paramètres DWA exposés
    struct DWAParams {
        float v_max;      // vitesse linéaire max
        float w_max;      // vitesse angulaire max
        float dt;         // pas de temps
        float T;          // horizon de prédiction
        float alpha;      // poids orientation
        float beta;       // poids vitesse
        float gamma;      // poids distance obstacle
        float epsilon;    // seuil de collision
        float dv;         // incrément vitesse linéaire
        float dw;         // incrément vitesse angulaire
        float sim_time;   // temps total de simulation
    } params;

private:
    // ========== MÉTHODES DWA ==========
    
    // Calcule la meilleure commande (v, w) selon DWA
    void CalcVelocityCommand(float &v_cmd, float &w_cmd);
    
    // Simule une trajectoire pour (v, w) donnés
    SimulatedTrajectory SimTrajectory(float v, float w, float dt, float T);
    
    // Calcule la distance minimale aux obstacles pour une trajectoire
    float MinimalDistance(const SimulatedTrajectory &traj);
    
    // Fonction de coût pour une trajectoire
    float EvaluateTrajectory(const SimulatedTrajectory &traj, float v);
    
    // Différence angulaire normalisée [-π, π]
    float AngDiff(float a, float b);
    
    // Avance le robot d'un pas de temps
    void RobotMotion(float &px, float &py, float &theta, float v, float w, float dt);

    // ========== TIMING ==========
    flair::core::Time previous_time;
    float CurrentTime;
    bool first_update;
    bool is_finishing;

    // ========== ÉTAT CINÉMATIQUE ==========
    flair::core::Vector2Df pos;      // position actuelle (x, y)
    flair::core::Vector2Df vel;      // vitesse actuelle (vx, vy)
    flair::core::Vector2Df acc;
    flair::core::Vector2Df jerk;
    flair::core::Vector2Df end_speed;

    // ========== OBSTACLES ==========
    std::vector<Obstacle> obstacles;
    bool have_end;

    // ========== UI WIDGETS ==========
    flair::gui::DoubleSpinBox *T;         // période (0 = auto)
    flair::gui::DoubleSpinBox *velocity;  // v_max
    flair::gui::DoubleSpinBox *angular;   // w_max
    flair::gui::DoubleSpinBox *alpha;     // poids orientation
    flair::gui::DoubleSpinBox *beta;      // poids vitesse
    flair::gui::DoubleSpinBox *gamma;     // poids obstacle
    flair::gui::DoubleSpinBox *epsilon;   // seuil collision
};

#endif // DWA2DTRAJECTORY_IMPL_H