#include "target_estimation/target_c.h"
#include "target_estimation/target_manager.hpp"
#include <Eigen/Core>



// Temporary variables
static Eigen::Vector7d vector7d_tmp_;
static Eigen::Vector6d vector6d_tmp_;

//C wrapper for TargetManager

extern "C" {
//constructor TargetManager
target_manager_c * target_manager_new(const char* file) {
    TargetManager *manager = new TargetManager(std::string(file));
    return (target_manager_c *)manager;
}

void target_manager_init(const target_manager_c *self, const unsigned int id, const double dt0, double p0[], const double t0) {
    TargetManager *manager = (TargetManager *)self;
    Eigen::Map<Eigen::Vector7d> p0_map(p0);
    manager->init(id,dt0,p0_map,t0);
}

void target_manager_update_meas(const target_manager_c *self, const unsigned int id, const double dt, double meas[]) {
    TargetManager *manager = (TargetManager *)self;
    Eigen::Map<Eigen::Vector7d> meas_map(meas);
    manager->update(id,dt,meas_map);
}

void target_manager_update(const target_manager_c *self, const unsigned int id, const double dt) {
    TargetManager *manager = (TargetManager *)self;
    manager->update(id,dt);
}

bool target_manager_get_int_pose(const target_manager_c *self, const unsigned int id, const double t, const double pos_th, const double ang_th, double int_pose[]) {
    TargetManager *manager = (TargetManager *)self;
    bool res;
    res = manager->getInterceptionPose(id,t,pos_th,ang_th,vector7d_tmp_);
    Eigen::Map<Eigen::Vector7d>(int_pose,7,1) = vector7d_tmp_;
    return res;
}

bool target_manager_get_est_pose(const target_manager_c *self, const unsigned int id, double pose[]) {
    TargetManager *manager = (TargetManager *)self;
    bool res;
    res = manager->getTargetPose(id,vector7d_tmp_);
    Eigen::Map<Eigen::Vector7d>(pose,7,1) = vector7d_tmp_;
    return res;
}

bool target_manager_get_est_twist(const target_manager_c *self, const unsigned int id, double twist[]) {
    TargetManager *manager = (TargetManager *)self;
    bool res;
    res = manager->getTargetTwist(id,vector6d_tmp_);
    Eigen::Map<Eigen::Vector6d>(twist,6,1) = vector6d_tmp_;
    return res;
}

bool target_manager_get_est_acceleration(const target_manager_c *self, const unsigned int id, double acceleration[]) {
    TargetManager *manager = (TargetManager *)self;
    bool res;
    res = manager->getTargetAcceleration(id,vector6d_tmp_);
    Eigen::Map<Eigen::Vector6d>(acceleration,6,1) = vector6d_tmp_;
    return res;
}

void target_manager_set_int_sphere(const target_manager_c *self, double origin[], double radius)
{
    TargetManager *manager = (TargetManager *)self;
    Eigen::Map<Eigen::Vector3d> origin_map(origin);
    return manager->setInterceptionSphere(origin_map,radius);
}

int target_manager_get_n_measurements(const target_manager_c *self, const unsigned int id)
{
    TargetManager *manager = (TargetManager *)self;
    return (int)manager->getNumberMeasurements(id);
}

void target_manager_log(const target_manager_c *self)
{
    TargetManager *manager = (TargetManager *)self;
    manager->log();
}

void target_manager_delete(target_manager_c *self) {
    TargetManager *manager = (TargetManager *)self;
    delete manager;
}
}
