/**
*
*/

#include "target_estimation/target_manager.hpp"
#include "target_estimation/types/rpy.hpp"
#include "target_estimation/types/rpyext.hpp"
#include "target_estimation/types/projectile.hpp"
#include <stdexcept>

using namespace std;

// ----------------------------
// Target Manager
// ----------------------------

bool TargetManager::parseSquareMatrix(const YAML::Node& node, const std::string& matrix, Eigen::MatrixXd& M)
{
    std::vector<double> Mv;
    try
    {
        Mv = node[matrix].as<std::vector<double>>();
        unsigned int size = static_cast<unsigned int>(std::sqrt(Mv.size()));
        M = Eigen::Map<Eigen::MatrixXd>(Mv.data(),size,size);
    }
    catch(YAML::ParserException& exception)
    {
        std::cerr << exception.what() << std::endl;
        return false;
    }
    return true;
}

bool TargetManager::parseTargetType(const YAML::Node& node, target_t& type)
{
    std::string type_str;
    try
    {
        type_str = node["type"].as<std::string>();
        if (std::strcmp(type_str.c_str(),"rpy")==0)
            type = TargetManager::target_t::RPY;
        else if (std::strcmp(type_str.c_str(),"rpy_ext") == 0)
            type = TargetManager::target_t::RPY_EXT;
        else if (std::strcmp(type_str.c_str(),"projectile") == 0)
            type = TargetManager::target_t::PROJECTILE;
    }
    catch(YAML::ParserException& exception)
    {
        std::cerr << exception.what() << std::endl;
        return false;
    }
    return true;
}

bool TargetManager::loadYamlFile(const std::string& file,
                                 Eigen::MatrixXd& Q,
                                 Eigen::MatrixXd& R,
                                 Eigen::MatrixXd& P,
                                 target_t& type)
{
    bool success = true;
    YAML::Node node;
    // Try to parse the yaml file
    try {
        node = YAML::LoadFile(file);
    } catch(YAML::ParserException& exception) {
        std::cerr << exception.what() << std::endl;
        success = false;
    }
    if(!parseSquareMatrix(node,"Q",Q))
    {
        std::cerr <<"Can not load matrix Q from file: "<<file<<std::endl;
        success = false;
    }
    if(!parseSquareMatrix(node,"R",R))
    {
        std::cerr <<"Can not load matrix R from file: "<<file<<std::endl;
        success = false;
    }
    if(!parseSquareMatrix(node,"P",P))
    {
        std::cerr <<"Can not load matrix P from file: "<<file<<std::endl;
        success = false;
    }
    if(!parseTargetType(node,type))
    {
        std::cerr <<"Can not load type from file: "<<file<<std::endl;
        success = false;
    }

    return success;
}

TargetManager::TargetManager()
{
    sphere_origin_ << 0.0, 0.0, 0.0;
    sphere_radius_ = 0.0;
    default_values_loaded_ = false;
}

TargetManager::TargetManager(const std::string& file)
    :TargetManager()
{
    if(!loadYamlFile(file,default_Q_,default_R_,default_P_,default_type_))
        throw "TargetManager default constructor failed!";
    else
        default_values_loaded_ = true;
}

void TargetManager::log()
{
    for (auto const& map : targets_)
      map.second->log();
}

std::vector<unsigned int> TargetManager::getAvailableTargets() const
{
  std::vector<unsigned int> ids;
  for (auto const& map : targets_)
    ids.push_back(map.first);
  return ids;
}

void TargetManager::init(const unsigned int& id, const double& dt0,
                         const Eigen::Vector7d& p0, const double& t0)
{
    if(default_values_loaded_)
        init(id,dt0,default_Q_,default_R_,default_P_,p0,t0,default_type_);
    else
        throw "TargetManager::init failed, can not find default values to load!";
}

void TargetManager::init(const unsigned int& id, const double& dt0,
                         const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, const Eigen::MatrixXd& P0,
                         const Eigen::Vector7d& p0, const double& t0, const target_t& type)
{
    lock_guard<mutex> lg(target_lock_);

    if(targets_.find(id) == targets_.end()) {
        // Target not found, create it
        switch(type)
        {
        case target_t::RPY:
            targets_[id].reset(new TargetRpy(id,dt0,Q,R,P0,p0,t0));
            std::cout << "Orientation defined as RPY angles" << std::endl;
            break;  
        case target_t::RPY_EXT:
            targets_[id].reset(new TargetRPYExtended(id,dt0,Q,R,P0,p0,t0));
            std::cout << "Orientation defined as RPY angles with omega" << std::endl;
            break;
        case target_t::PROJECTILE:
            targets_[id].reset(new TargetProjectile(id,dt0,Q,R,P0,p0,t0));
            std::cout << "Catching some bullets! No orientation needed" << std::endl;
            break;
        }
    }
    else
        std::cout<<"Target("<<id<<") already exists!"<<std::endl;
}

void TargetManager::init(const std::string& file, const unsigned int& id, const double& dt0,
                         const Eigen::Vector7d& p0, const double& t0)
{
    Eigen::MatrixXd Q, P, R;
    target_t type;
    loadYamlFile(file,Q,R,P,type);
    init(id,dt0,Q,R,P,p0,t0,type);
}

void TargetManager::update(const unsigned int& id, const double& dt, const Eigen::Vector7d& meas)
{
    lock_guard<mutex> lg(target_lock_);
    if(targets_.find(id) == targets_.end()) {
        // Target not found, skip
        std::cout<<"Target("<<id<<") does not exist!"<<std::endl;
        return;
    }
    // Add a measurement for Target
    // and update (predict + estimate) the filters
    targets_[id]->addMeasurement(dt,meas);
}

void TargetManager::update(const unsigned int& id, const double& dt)
{
    lock_guard<mutex> lg(target_lock_);
    if(targets_.find(id) == targets_.end()) {
        // Target not found, skip
        std::cout<<"Target("<<id<<") does not exist!"<<std::endl;
        return;
    }
    else
    {
        // Perform only the predict step
        targets_[id]->update(dt);
    }
}

void TargetManager::update(const double& dt)
{
    lock_guard<mutex> lg(target_lock_);
    for(const auto& tmp : targets_)
      tmp.second->update(dt);
}

void TargetManager::setInterceptionSphere(const Eigen::Vector3d& origin, const double& radius)
{
    assert(radius>=0.0);
    lock_guard<mutex> lg(target_lock_);
    sphere_origin_ = origin;
    sphere_radius_ = radius;
}

TargetInterface::Ptr TargetManager::getTarget(const unsigned int& id)
{
    lock_guard<mutex> lg(target_lock_);
    if (targets_.count(id)!=0)
        return targets_[id];
    else
        return nullptr;
}

bool TargetManager::getTargetPose(const unsigned int& id, Eigen::Vector7d& pose)
{
    if(getTarget(id)!=nullptr)
    {
        pose = getTarget(id)->getEstimatedPose();
        return true;
    }
    else
        return false;
}

bool TargetManager::getTargetOrientation(const unsigned int& id, Eigen::Quaterniond& orientation)
{
    if(getTarget(id)!=nullptr)
    {
        orientation = getTarget(id)->getEstimatedOrientation();
        return true;
    }
    else
        return false;
}

bool TargetManager::getTargetOrientation(const unsigned int& id, Eigen::Vector3d& orientation)
{
    if(getTarget(id)!=nullptr)
    {
        orientation = getTarget(id)->getEstimatedRPY();
        return true;
    }
    else
        return false;
}

bool TargetManager::getTargetTwist(const unsigned int& id, Eigen::Vector6d& twist)
{
    if(getTarget(id)!=nullptr)
    {
        twist = getTarget(id)->getEstimatedTwist();
        return true;
    }
    else
        return false;
}

bool TargetManager::getTargetAcceleration(const unsigned int& id, Eigen::Vector6d& acc)
{
    if(getTarget(id)!=nullptr)
    {
        acc = getTarget(id)->getEstimatedAcceleration();
        return true;
    }
    else
        return false;
}

unsigned int TargetManager::getClosestInterceptionPose(const double& t1, const double& pos_th, const double& ang_th, Eigen::Vector7d& interception_pose)
{
    lock_guard<mutex> lg(target_lock_);
    bool res = false;
    unsigned int closest_target_id = 0;
    double smallest_intersection_time = 10000.0; // Dummy value
    for(const auto& tmp : targets_)
    {
      double current_intersection_time = tmp.second->getIntersectionTime(t1,sphere_origin_,sphere_radius_);
      if(current_intersection_time < smallest_intersection_time)
      {
        closest_target_id = tmp.first;
        tmp.second->getIntersectionPose(t1,pos_th,ang_th,sphere_origin_,sphere_radius_,interception_pose);
      }
    }
    return closest_target_id;
}

bool TargetManager::getInterceptionPose(const unsigned int& id, const double& t1, const double& pos_th, const double& ang_th, Eigen::Vector7d& interception_pose)
{
    lock_guard<mutex> lg(target_lock_);
    bool res = false;
    if (targets_.count(id)!=0)
        res = targets_[id]->getIntersectionPose(t1,pos_th,ang_th,sphere_origin_,sphere_radius_,interception_pose);
    else
        std::cout<<"Target("<<id<<") does not exist!"<<std::endl;

    //if(res)
    //    std::cout<<"Target("<<id<<") converged!"<<std::endl;

    return res;
}

long long TargetManager::getNumberMeasurements(const unsigned int& id)
{
    lock_guard<mutex> lg(target_lock_);
    if (targets_.count(id)!=0)
        return targets_[id]->getNumberMeasurements();
    else
    {
        std::cout<<"Target("<<id<<") does not exist!"<<std::endl;
        return 0;
    }
}
