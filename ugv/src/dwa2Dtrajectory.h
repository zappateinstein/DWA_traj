#ifndef DWA2DTRAJECTORY_H
#define DWA2DTRAJECTORY_H

#include <IODevice.h>
#include <Vector2D.h>
#include <vector>

// Forward declaration
struct Obstacle;

namespace flair {
namespace core {
class Matrix;
}
namespace gui {
class LayoutPosition;
}
}

class dwa2Dtrajectory_impl;

namespace flair {
namespace filter {

/*!
 * \class dwa2Dtrajectory
 * \brief Générateur de trajectoire 2D avec algorithme DWA
 *
 * Utilise Dynamic Window Approach pour générer des trajectoires 
 * évitant les obstacles. Sorties: pos(x,y), vel(x,y), acc(x,y), jerk(x,y).
 */
class dwa2Dtrajectory : public core::IODevice {
public:
    /**
     * Constructeur
     * @param position Position du widget dans l'interface
     * @param name Nom du générateur
     */
    dwa2Dtrajectory(const gui::LayoutPosition *position, std::string name1);
    ~dwa2Dtrajectory();

    // ========== CONTRÔLE DE TRAJECTOIRE ==========
    
    /**
     * Démarre la trajectoire depuis une position donnée
     * @param start_pos Position de départ (position actuelle du robot)
     */
    void StartTraj(const core::Vector2Df &start_pos);
    
    /**
     * Termine la trajectoire (décélération progressive)
     */
    void FinishTraj(void);
    
    /**
     * Arrête immédiatement la trajectoire (pause)
     */
    void StopTraj(void);

    // ========== GESTION DES OBSTACLES ==========
    
    /**
     * Définit la liste complète des obstacles
     * @param obs Vecteur d'obstacles
     */
    void SetObstacles(const std::vector<Obstacle> &obs);
    
    /**
     * Ajoute un obstacle ponctuel
     * @param x Position X de l'obstacle
     * @param y Position Y de l'obstacle
     * @param radius Rayon de sécurité autour de l'obstacle (défaut: 0.3m)
     */
    void AddObstacle(float x, float y, float radius = 0.3f);
    
    /**
     * Efface tous les obstacles
     */
    void ClearObstacles();

    // ========== SETTERS/GETTERS ==========
    
    /**
     * Définit la position cible (goal)
     */
    void SetEnd(const core::Vector2Df &value);
    void GetEnd(core::Vector2Df &point) const;
    
    /**
     * Définit la vitesse finale souhaitée
     */
    void SetEndSpeed(const core::Vector2Df &value);

    /**
     * Mise à jour de la trajectoire (appelée périodiquement)
     */
    void Update(core::Time time);

    /**
     * Récupère la position actuelle générée
     */
    void GetPosition(core::Vector2Df &point) const;
    
    /**
     * Récupère la vitesse actuelle générée
     */
    void GetSpeed(core::Vector2Df &point) const;
    
    /**
     * Récupère l'accélération actuelle
     */
    void GetAcceleration(core::Vector2Df &point) const;
    
    /**
     * Récupère le jerk (dérivée de l'accélération)
     */
    void GetJerk(core::Vector2Df &point) const;

    /**
     * @return Matrice de sortie contenant pos, vel, acc, jerk
     */
    core::Matrix *GetMatrix(void) const;
    
    /**
     * @return true si la trajectoire est en cours d'exécution
     */
    bool IsRunning(void) const;

private:
    void UpdateFrom(const core::io_data *data) override {}

    dwa2Dtrajectory_impl *pimpl_;  // Implementation (pimpl idiom)
};

} // namespace filter
} // namespace flair

#endif // DWA2DTRAJECTORY_H
