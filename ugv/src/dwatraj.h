//  created:    2020/12/09
//  filename:   dwatraj.h
// ... header info ...

#ifndef DWATRAJ_H
#define DWATRAJ_H

#include <Thread.h>
#include <Vector2D.h>
#include <vector>
#include <netinet/in.h>

namespace flair {
    namespace gui {
        class PushButton;
        class DoubleSpinBox;
    }
    namespace filter {
        class Pid;
        class dwa2Dtrajectory; // Forward declaration required here
    }
    namespace meta {
        class MetaVrpnObject;
    }
    namespace sensor {
        class TargetController;
    }
}

class dwatraj : public flair::core::Thread {
    public:
        dwatraj(std::string name, flair::sensor::TargetController *controller);
        ~dwatraj();

    private:
        enum class BehaviourMode_t {
            Manual,
            Auto
        };

        void Run(void) override; 
        
        // Standardized names
        void StartTraj(void); 
        void StopTraj(void);
        
        void ComputeManualControls(void);
        void ComputeAutoControls(void); // Renamed from ComputeCircleControls for clarity
        void SecurityCheck(void);
        void CheckJoystick(void);
        void CheckPushButton(void);
        void InitializeObstacles(int nb_obs);  // Nouvelle méthode

        flair::filter::Pid *uX, *uY;
        flair::gui::PushButton *startTraj, *stopTraj, *quitProgram, *startLog, *stopLog, *Obstacles;
        flair::gui::DoubleSpinBox *l;
        flair::meta::MetaVrpnObject *targetVrpn, *ugvVrpn;
        std::vector<flair::meta::MetaVrpnObject *> obstaclesVrpn;
        flair::filter::dwa2Dtrajectory *trajectory;
        BehaviourMode_t behaviourMode;
        bool vrpnLost;
        flair::sensor::TargetController *controller;

        int sockfd;
        struct sockaddr_in gc_addr;
        struct sockaddr_in python_addr;  // Pour le script Python
};

#endif // DWATRAJ_H