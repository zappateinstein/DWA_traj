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

using namespace std;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::sensor;
using namespace flair::filter;
using namespace flair::meta;
using namespace flair::actuator;

// ========== CONSTRUCTOR ==========
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
    std::cerr << "[dwa_path] Initial obstacles configured\n";

    // ========== Initial Goal Configuration ==========
    Vector2Df initial_goal(2.0f, 3.0f);
    trajectory->SetEnd(initial_goal);
    std::cerr << "[dwa_path] Initial goal set to (" << initial_goal.x 
              << ", " << initial_goal.y << ")\n";
    
    trajectory->ClearObstacles();
    trajectory->AddObstacle(1.5f, 1.5f, 0.3f);
    trajectory->AddObstacle(3.0f, 3.5f, 0.4f);
    trajectory->AddObstacle(0.8f, 4.2f, 0.25f);

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
    std::cerr << "[dwa_path] Initialization complete\n";
}

dwatraj::~dwatraj() {
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
      StartTraj(); // Corrected name
        
  if (stopTraj->Clicked() == true)
      StopTraj(); // Corrected name

  if (quitProgram->Clicked() == true)
      SafeStop();
}

// ========== JOYSTICK CHECK ==========
void dwatraj::CheckJoystick(void) {
  // R1 and Start -> Start Trajectory
  if(controller->ButtonClicked(4) && controller->IsButtonPressed(9)) {
      StartTraj();
  }

  // R1 and Cross -> Stop Trajectory
  if(controller->ButtonClicked(5) && controller->IsButtonPressed(9)) {
      StopTraj();
  }
}

// ========== SECURITY ==========
void dwatraj::SecurityCheck(void) {
    if ((!vrpnLost) && (behaviourMode == BehaviourMode_t::Auto)) {
        // Warning: if you don't use 'targetVrpn' anymore, you can remove this check
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
  
  // DWA Update
  trajectory->Update(GetTime());
  trajectory->GetPosition(traj_pos);
  trajectory->GetSpeed(traj_vel);
  trajectory->GetEnd(goal_pos);

  // Error calculation
  pos_error = ugv_2Dpos - traj_pos; // Warning: Sign might need inversion (traj - ugv) for standard PID
  vel_error = ugv_2Dvel - traj_vel;
  
  // Note: Usually PID error is (Target - Current). 
  // If your PID gains are positive, you might want (traj_pos - ugv_2Dpos).
  // I kept your sign logic, assuming your PID or actuator handles it.
  
  uX->SetValues(pos_error.x, vel_error.x);
  uX->Update(GetTime());
  uY->SetValues(pos_error.y, vel_error.y);
  uY->Update(GetTime());
  
  // Calculate distance to REAL goal
  float real_dist_to_goal = (goal_pos - ugv_2Dpos).GetNorm();

  // Stop Condition
  if (!trajectory->IsRunning() && real_dist_to_goal < 0.1f) {
    std::cout << "[dwa_path] Goal reached (Physics)! Stopping.\n";
    GetUgv()->GetUgvControls()->SetControls(0, 0);
    std::cerr << "[dwa_path] Started DWA from (" << ugv_2Dpos.x << ", " 
                  << ugv_2Dpos.y << ") toward goal (" << goal_pos.x << ", " 
                  << goal_pos.y << ")\n";
    behaviourMode = BehaviourMode_t::Manual;
    return;
  }

  // Get Yaw
  Quaternion vrpnQuaternion;
  ugvVrpn->GetQuaternion(vrpnQuaternion);
  float yaw = vrpnQuaternion.ToEuler().yaw;
  
  // Control Mix
  float v = cosf(yaw) * uX->Output() + sinf(yaw) * uY->Output();
  float w = -sinf(yaw) / l->Value() * uX->Output() + cosf(yaw) / l->Value() * uY->Output();
  std::cerr << "[dwa_path] Started DWA from (" << ugv_2Dpos.x << ", " 
                  << ugv_2Dpos.y << ") toward goal (" << goal_pos.x << ", " 
                  << goal_pos.y << ")\n";
  // Send Controls
  GetUgv()->GetUgvControls()->SetControls(-2, -2);
}

// ========== START TRAJECTORY ==========
void dwatraj::StartTraj(void) {
  if(behaviourMode != BehaviourMode_t::Auto) {
    Vector3Df ugv_pos;
    Vector2Df ugv_2Dpos;

    ugvVrpn->GetPosition(ugv_pos);
    ugv_pos.To2Dxy(ugv_2Dpos);
    
    // Set Goal
    Vector2Df goal(5.0f, 5.0f);
    trajectory->SetEnd(goal);
    
    // Start DWA from current robot position
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
    trajectory->FinishTraj(); // Or StopTraj() depending on what you want
    behaviourMode = BehaviourMode_t::Manual;
    GetUgv()->GetUgvControls()->SetControls(0, 0);
    Thread::Info("DWA Controller: Stopping Auto Mode\n");
  }
}