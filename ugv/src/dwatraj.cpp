//  created:    2020/12/09
//  filename:   CircleFollower.cpp
//
//  author:     Guillaume Sanahuja
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  version:    $Id: $
//
//  purpose:    demo cercle avec optitrack
//
//
/*********************************************************************/

#include "dwatraj.h"
#include <TargetController.h>
#include <GridLayout.h>
#include <PushButton.h>
#include <DataPlot1D.h>
#include <DataPlot2D.h>
#include <FrameworkManager.h>
#include <VrpnClient.h>
#include <MetaVrpnObject.h>
#include "dwa2Dtrajectory.h"
#include <Matrix.h>
#include <Tab.h>
#include <TabWidget.h>
#include <DoubleSpinBox.h>
#include <Pid.h>
#include <Quaternion.h>
#include <Euler.h>
#include <Ugv.h>
#include <UgvControls.h>
#include <math.h>
#include <iostream>
#include <string>
#include <vector>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sstream>
#include <cstring>

using namespace std;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::sensor;
using namespace flair::filter;
using namespace flair::meta;
using namespace flair::actuator;

// =========================================================
// LISTE MANUELLE D'OBSTACLES (IDENTIQUE AU SIMULATEUR)
// =========================================================
float obs_coords_ctrl[][2] = {
    // 1. Un mur vertical en X=1.5 (Force le détour)
    {1.5f, -1.0f}, {1.5f, 0.0f}, {1.5f, 1.0f}, {1.5f, 2.0f},
    
    // 2. Un mur horizontal en Y=3.0 (Barrière du haut)
    {-1.0f, 3.0f}, {0.0f, 3.0f}, {1.0f, 3.0f},
    
    // 3. Un "Goulot" étroit en (3.5, 2.0)
    {-3.5f, 1.5f}, {3.5f, 3.5f}, 
    
    // 4. Protection autour du but (supposé vers 5,5)
    {4.5f, 4.0f}, {-4.0f, 5.5f},
    
    // 5. Obstacles dispersés (Pièges)
    {2.0f, 2.0f}, {2.0f, 4.0f}, {2.5f, -1.5f}, {5.0f, 0.0f}
};
// Nombre d'obstacles
int nb_obs_ctrl = 15;
// =========================================================

dwatraj::dwatraj(string name, TargetController *controller): Thread(getFrameworkManager(), "DWA_Controller", 50), behaviourMode(BehaviourMode_t::Manual), vrpnLost(false) {
    this->controller = controller;
    controller->Start();
    
    Ugv* ugv = GetUgv();
    ugv->UseDefaultPlot();
    
    VrpnClient* vrpnclient = new VrpnClient("vrpn", ugv->GetDefaultVrpnAddress(), 80);
    ugvVrpn = new MetaVrpnObject(name);
    targetVrpn = new MetaVrpnObject("target");
    
    getFrameworkManager()->AddDeviceToLog(ugvVrpn);
    getFrameworkManager()->AddDeviceToLog(targetVrpn);
    vrpnclient->Start();
  
    Tab *ugvTab = new Tab(getFrameworkManager()->GetTabWidget(), "ugv", 0);
    GridLayout* buttonslayout = new GridLayout(ugvTab->NewRow(), "buttons");
    quitProgram = new PushButton(buttonslayout->NewRow(), "quit program");
    startTraj = new PushButton(buttonslayout->NewRow(), "start_traj");
    stopTraj = new PushButton(buttonslayout->LastRowLastCol(), "stop_traj");
    startLog = new PushButton(buttonslayout->NewRow(), "start_log");
    stopLog = new PushButton(buttonslayout->LastRowLastCol(), "stop_log");
    
    trajectory = new dwa2Dtrajectory(vrpnclient->GetLayout()->NewRow(), "Kinematic");
    std::cout << "création du chemin suivi par le robot\n";
    std::cerr << "[DWA_traj] Initial obstacles configured\n";

    // ========== Initial Goal Configuration ==========
    // On met le but à l'opposé pour traverser le champ de mines
    Vector2Df initial_goal(5.0f, 5.0f);
    trajectory->SetEnd(initial_goal);
    std::cerr << "[DWA_traj] Initial goal set to (" << initial_goal.x 
              << ", " << initial_goal.y << ")\n";
    
    // ========== INJECTION DES OBSTACLES DWA ==========
    trajectory->ClearObstacles();
    
    for(int i = 0; i < nb_obs_ctrl; i++) {
        float x = obs_coords_ctrl[i][0];
        float y = obs_coords_ctrl[i][1];
        
        // Ajout avec un rayon de sécurité de 0.4m (Ball = 0.3 + Marge)
        trajectory->AddObstacle(x, y, 0.1f);
        
        std::cerr << " [DWA] Obstacle ajouté: (" << x << ", " << y << ")\n";
    }
    // =================================================

    ugvVrpn->xPlot()->AddCurve(trajectory->GetMatrix()->Element(0,0), DataPlot::Blue);
    ugvVrpn->yPlot()->AddCurve(trajectory->GetMatrix()->Element(0,1), DataPlot::Blue);
    ugvVrpn->VxPlot()->AddCurve(trajectory->GetMatrix()->Element(1,0), DataPlot::Blue);
    ugvVrpn->VyPlot()->AddCurve(trajectory->GetMatrix()->Element(1,1), DataPlot::Blue);
    ugvVrpn->XyPlot()->AddCurve(trajectory->GetMatrix()->Element(0,1), trajectory->GetMatrix()->Element(0,0), DataPlot::Blue, "dwa_trajectory");

    Tab *lawTab = new Tab(getFrameworkManager()->GetTabWidget(), "control laws");
    TabWidget *tabWidget = new TabWidget(lawTab->NewRow(), "laws");
    Tab *setupLawTab = new Tab(tabWidget, "Setup");
    Tab *graphLawTab = new Tab(tabWidget, "Graphes");
    uX = new Pid(setupLawTab->At(1,0), "u_x");
    uX->UseDefaultPlot(graphLawTab->NewRow());
    uY = new Pid(setupLawTab->At(1,1), "u_y");
    uY->UseDefaultPlot(graphLawTab->LastRowLastCol());
    
    getFrameworkManager()->AddDeviceToLog(uX);
    getFrameworkManager()->AddDeviceToLog(uY);

    l = new DoubleSpinBox(setupLawTab->NewRow(), "L", " m", 0, 10, 0.1, 1, 1);
    std::cerr << "[DWA_traj] Initialization complete\n";

    // Setup UDP
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        perror("socket creation failed");
    }
    memset(&gc_addr, 0, sizeof(gc_addr));
    gc_addr.sin_family = AF_INET;
    gc_addr.sin_port = htons(9005);
    gc_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
}

dwatraj::~dwatraj() {
    if (sockfd >= 0) {
        close(sockfd);
    }
}

void dwatraj::Run(void) {
    WarnUponSwitches(true);
    SetPeriodMS(20);
    
    if (getFrameworkManager()->ErrorOccured() == true) {
        SafeStop();
    }

    while (!ToBeStopped()) {
        SecurityCheck();
        CheckJoystick();
        CheckPushButton();
       
        if(behaviourMode == BehaviourMode_t::Manual) ComputeManualControls();
        if(behaviourMode == BehaviourMode_t::Auto) ComputeAutoControls();

        // --- TELEMETRY (Always send) ---
        Vector3Df t_ugv_pos, t_ugv_vel;
        Vector2Df t_ugv_2Dpos, t_ugv_2Dvel;
        Vector2Df t_goal_pos;
        
        ugvVrpn->GetPosition(t_ugv_pos);
        ugvVrpn->GetSpeed(t_ugv_vel);
        t_ugv_pos.To2Dxy(t_ugv_2Dpos);
        t_ugv_vel.To2Dxy(t_ugv_2Dvel);
        
        Quaternion t_vrpnQuaternion;
        ugvVrpn->GetQuaternion(t_vrpnQuaternion);
        float t_yaw = t_vrpnQuaternion.ToEuler().yaw;

        trajectory->GetEnd(t_goal_pos);
        
        std::stringstream ss;
        // DWA,timestamp,ugv_x,ugv_y,ugv_yaw,ugv_vx,ugv_vy,target_x,target_y,obs_count,[obs_x,obs_y,obs_r...]
        ss << "DWA," << GetTime() / 1000000000.0 << ",";
        ss << t_ugv_2Dpos.x << "," << t_ugv_2Dpos.y << "," << t_yaw << ",";
        ss << t_ugv_2Dvel.x << "," << t_ugv_2Dvel.y << ",";
        ss << t_goal_pos.x << "," << t_goal_pos.y << ",";
        
        ss << nb_obs_ctrl;
        for(int i=0; i<nb_obs_ctrl; i++) {
            ss << "," << obs_coords_ctrl[i][0] << "," << obs_coords_ctrl[i][1] << "," << 0.1;
        }

        std::string msg = ss.str();
        sendto(sockfd, msg.c_str(), msg.length(), 0, (const struct sockaddr *)&gc_addr, sizeof(gc_addr));
        // --------------------------------

        WaitPeriod();
    }
}

// ========== BUTTONS CHECK ==========
void dwatraj::CheckPushButton(void) {
  if (startLog->Clicked() == true)
    getFrameworkManager()->StartLog();
  if (stopLog->Clicked() == true)
    getFrameworkManager()->StopLog();
  
  if (startTraj->Clicked() == true)
      StartTraj(); 
  if (stopTraj->Clicked() == true)
      StopTraj(); 
  if (quitProgram->Clicked() == true)
      SafeStop();
}

// ========== JOYSTICK CHECK ==========
void dwatraj::CheckJoystick(void) {
  if(controller->ButtonClicked(4) && controller->IsButtonPressed(9)) {
      StartTraj();
  }
  if(controller->ButtonClicked(5) && controller->IsButtonPressed(9)) {
      StopTraj();
  }
}

// ========== SECURITY ==========
void dwatraj::SecurityCheck(void) {
    if ((!vrpnLost) && (behaviourMode == BehaviourMode_t::Auto)) {
        if (!ugvVrpn->IsTracked(500)) {
            Thread::Err("VRPN, ugv lost\n");
            vrpnLost = true;
            StopTraj();
        }
    }
}

// ========== MANUAL CONTROL ==========
void dwatraj::ComputeManualControls(void) {
  float speed = -controller->GetAxisValue(3);
  float turn = controller->GetAxisValue(0);
  GetUgv()->GetUgvControls()->SetControls(speed, turn);
}

// ========== AUTO CONTROL (DWA FOLLOWER) ==========
void dwatraj::ComputeAutoControls(void) {
  Vector3Df ugv_pos, ugv_vel;
  Vector2Df ugv_2Dpos, ugv_2Dvel;
  Vector2Df pos_error, vel_error;
  Vector2Df traj_pos, traj_vel;
  Vector2Df goal_pos;
    
  ugvVrpn->GetPosition(ugv_pos);
  ugvVrpn->GetSpeed(ugv_vel);

  ugv_pos.To2Dxy(ugv_2Dpos);
  ugv_vel.To2Dxy(ugv_2Dvel);
  
  // Note: On n'a plus besoin de mettre à jour les obstacles ici 
  // car ils sont statiques et déjà chargés dans le constructeur.
  
  // DWA Update
  trajectory->Update(GetTime());
  trajectory->GetPosition(traj_pos);
  trajectory->GetSpeed(traj_vel);
  trajectory->GetEnd(goal_pos);

  pos_error = ugv_2Dpos - traj_pos; 
  vel_error = ugv_2Dvel - traj_vel;
  
  uX->SetValues(pos_error.x, vel_error.x);
  uX->Update(GetTime());
  uY->SetValues(pos_error.y, vel_error.y);
  uY->Update(GetTime());
  
  float real_dist_to_goal = (goal_pos - ugv_2Dpos).GetNorm();

  if (!trajectory->IsRunning() && real_dist_to_goal < 0.1f) {
    std::cout << "[DWA_traj] Goal reached (Physics)! Stopping.\n";
    GetUgv()->GetUgvControls()->SetControls(0, 0);
    behaviourMode = BehaviourMode_t::Manual;
    return;
  }

  Quaternion vrpnQuaternion;
  ugvVrpn->GetQuaternion(vrpnQuaternion);
  float yaw = vrpnQuaternion.ToEuler().yaw;
  
  float v = cosf(yaw) * uX->Output() + sinf(yaw) * uY->Output();
  float w = -sinf(yaw) / l->Value() * uX->Output() + cosf(yaw) / l->Value() * uY->Output();
  
  GetUgv()->GetUgvControls()->SetControls(-v, -w);
}

// ========== START TRAJECTORY ==========
void dwatraj::StartTraj(void) {
  if(behaviourMode != BehaviourMode_t::Auto) {
    Vector3Df ugv_pos;
    Vector2Df ugv_2Dpos;

    ugvVrpn->GetPosition(ugv_pos);
    ugv_pos.To2Dxy(ugv_2Dpos);
    
    // Position de départ (ex: -5, -5)
    // Goal déjà configuré dans le constructeur (5, 5) ou réinitialisez-le ici
    trajectory->StartTraj(ugv_2Dpos);

    uX->Reset();
    uY->Reset();
    behaviourMode = BehaviourMode_t::Auto;
    Thread::Info("DWA Controller: Starting Auto Mode\n");
  }
}

// ========== STOP TRAJECTORY ==========
void dwatraj::StopTraj(void) {
  if(behaviourMode == BehaviourMode_t::Auto) {
    trajectory->FinishTraj(); 
    behaviourMode = BehaviourMode_t::Manual;
    GetUgv()->GetUgvControls()->SetControls(0, 0);
    Thread::Info("DWA Controller: Stopping Auto Mode\n");
  }
}