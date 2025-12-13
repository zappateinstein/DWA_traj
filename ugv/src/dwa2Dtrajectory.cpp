#include "dwa2Dtrajectory.h"
#include "dwa2Dtrajectory_impl.h"
#include <Matrix.h>
#include <Layout.h>
#include <LayoutPosition.h>
#include <Vector2D.h>
#include <DoubleSpinBox.h>

using std::string;
using namespace flair::core;
using namespace flair::gui;

namespace flair {
namespace filter {

dwa2Dtrajectory::dwa2Dtrajectory(
    const LayoutPosition *position, string name1)
    : IODevice(position->getLayout(), name1) {
    
    // Création de l'implémentation avec 2 groupbox :
    // - name1 pour la partie motion
    // - name2 pour la partie costs
    pimpl_ = new dwa2Dtrajectory_impl(this, position,
                                      name1);
    
    AddDataToLog(pimpl_->output);
    SetIsReady(true);
}

dwa2Dtrajectory::~dwa2Dtrajectory() {
    delete pimpl_;
}

// ========== CONTRÔLE ==========

void dwa2Dtrajectory::StartTraj(const Vector2Df &start_pos) {
    pimpl_->StartTraj(start_pos);
}

void dwa2Dtrajectory::FinishTraj(void) {
    pimpl_->FinishTraj();
}

void dwa2Dtrajectory::StopTraj(void) {
    pimpl_->StopTraj();
}

// ========== OBSTACLES ==========

void dwa2Dtrajectory::SetObstacles(const std::vector<Obstacle> &obs) {
    pimpl_->SetObstacles(obs);
}

void dwa2Dtrajectory::AddObstacle(float x, float y, float radius) {
    pimpl_->AddObstacle(x, y, radius);
}

void dwa2Dtrajectory::ClearObstacles() {
    pimpl_->ClearObstacles();
}

// ========== SETTERS/GETTERS ==========

void dwa2Dtrajectory::SetEnd(const Vector2Df &value) {
    pimpl_->SetEnd(value);
}

void dwa2Dtrajectory::GetEnd(Vector2Df &point) const {
    pimpl_->GetEnd(point);
}

void dwa2Dtrajectory::SetEndSpeed(const Vector2Df &value) {
    pimpl_->SetEndSpeed(value);
}

void dwa2Dtrajectory::GetPosition(Vector2Df &point) const {
    pimpl_->GetPos(point);
}

void dwa2Dtrajectory::GetSpeed(Vector2Df &point) const {
    pimpl_->GetVel(point);
}

void dwa2Dtrajectory::GetAcceleration(Vector2Df &point) const {
    pimpl_->GetAcc(point);
}

void dwa2Dtrajectory::GetJerk(Vector2Df &point) const {
    pimpl_->GetJerk(point);
}

// ========== UPDATE & STATE ==========

void dwa2Dtrajectory::Update(Time time) {
    pimpl_->Update(time);
    ProcessUpdate(pimpl_->output);
}

Matrix *dwa2Dtrajectory::GetMatrix(void) const {
    return pimpl_->output;
}

bool dwa2Dtrajectory::IsRunning(void) const {
    return pimpl_->is_running;
}

} // namespace filter
} // namespace flair