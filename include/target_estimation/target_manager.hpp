/**
*
*/

#ifndef TARGET_MANAGER_HPP
#define TARGET_MANAGER_HPP

#include "target_estimation/target.hpp"
#include <yaml-cpp/yaml.h>

// ----------------------------
/**
 * @brief The TargetManager class
 */
class TargetManager {

public:

    typedef std::map<unsigned int,TargetInterface::Ptr> targets_map_t;

    enum target_t {RPY=0,RPY_EXT};

    /**
   * @brief TargetManager Create an empty TargetManager
   */
    TargetManager();

    /**
   * @brief TargetManager Create a TargetManager by loading
   * a YAML file with default covariance matrices and target type.
   * @param file
   */
    TargetManager(const std::string& file);

    /**
   * @brief init  Initialize a new target with default covariance matrices and type
   * @param id    Target's id
   * @param dt0   Estimated initial sample time
   * @param p0    Initial estimation of the target's position
   *              Note: we assume p0 = [x y z qx qy qz qw] i.e. size(p0) = 7
   * @param t0    Initial time
   */
    void init(const unsigned int& id, const double& dt0,
              const Eigen::Vector7d& p0, const double& t0 = 0.0);

    /**
   * @brief init  Initialize a new target
   * @param id    Target's id
   * @param dt0   Estimated initial sample time
   * @param Q     Process noise covariance
   * @param R     Measurement noise covariance
   * @param P0    Initial estimation of the error covariance matrix
   * @param p0    Initial estimation of the target's position
   *              Note: we assume p0 = [x y z qx qy qz qw] i.e. size(p0) = 7
   * @param t0    Initial time
   * @param type  Kalman Filter type
   */
    void init(const unsigned int& id, const double& dt0,
              const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, const Eigen::MatrixXd& P0,
              const Eigen::Vector7d& p0, const double& t0 = 0.0, const target_t& type = target_t::RPY);

    /**
   * @brief init  Initialize a new target
   * @param file  YAML file path with the covariance matrices and target type
   * @param id    Target's id
   * @param dt0   Estimated initial sample time
   * @param p0    Initial estimation of the target's position
   *              Note: we assume p0 = [x y z qx qy qz qw] i.e. size(p0) = 7
   * @param t0    Initial time
   */
    void init(const std::string& file, const unsigned int& id, const double& dt0,
              const Eigen::Vector7d& p0, const double& t0 = 0.0);
    /**
   * @brief update Add a measurement to the given Target ID. Perform an update step with the filters.
   * @param id
   * @param dt
   * @param meas
   */
    void update(const unsigned int& id, const double& dt, const Eigen::Vector7d& meas);

    /**
   * @brief update perform a predict step with the filters.
   * @param id
   * @param dt
   */
    void update(const unsigned int& id, const double& dt);

    /**
   * @brief setInterceptionSphere Set the interception sphere params
   * @param origin
   * @param radius
   */
    void setInterceptionSphere(const Eigen::Vector3d& origin, const double& radius);

    /**
   * @brief getTarget Return a specific target, a nullptr is returned if the selected target does not exist
   * @param id
   * @return
   */
    TargetInterface::Ptr getTarget(const unsigned int& id);

    /**
   * @brief getTargetOrientation Output a specific target estimated orientation represetend as a quaternion
   * @param id
   * @param orientation
   * @return false if the selected target does not exist
   */
    bool getTargetOrientation(const unsigned int& id, Eigen::Quaterniond& orientation);

    /**
   * @brief getTargetOrientation Output a specific target estimated orientation represetend as a rpy
   * @param id
   * @param orientation
   * @return false if the selected target does not exist
   */
    bool getTargetOrientation(const unsigned int& id, Eigen::Vector3d& orientation);

    /**
   * @brief getTargetPose Output a specific target estimated position
   * @param id
   * @param pose
   * @return false if the selected target does not exist
   */
    bool getTargetPose(const unsigned int& id, Eigen::Vector7d& pose);

    /**
   * @brief getTargetTwist Output a specific target estimated twist
   * @param id
   * @param twist
   * @return false if the selected target does not exist
   */
    bool getTargetTwist(const unsigned int& id, Eigen::Vector6d& twist);

    /**
   * @brief getTargetAcceleration Output a specific target estimated acceleration
   * @param id
   * @param acc
   * @return false if the selected target does not exist
   */
    bool getTargetAcceleration(const unsigned int& id, Eigen::Vector6d& acc);

    /**
   * @brief getInterceptionPose Compute the interception pose for the robot to capture the target
   * @param id
   * @param t
   * @param pos_th defines the convergence threshold for the interception position
   * @param ang_th defines the convergence threshold for the interception orientation
   * @param interception_pose
   * @return true if there is a valid interception pose
   */
    bool getInterceptionPose(const unsigned int& id, const double& t, const double& pos_th, const double& ang_th, Eigen::Vector7d& interception_pose);

    /**
   * @brief getNumberMeasurements Return the number of measurements done so far
   * @return
   */
    long long getNumberMeasurements(const unsigned int& id);

    /**
   * @brief log Log data through ros
   */
    void log();

    /**
   * @brief get the target ids
   */
    std::vector<unsigned int> getAvailableTargets() const;

private:

    /**
   * @brief parseSquareMatrix
   * @param node
   * @param matrix
   * @param M
   * @return
   */
    bool parseSquareMatrix(const YAML::Node& node, const std::string& matrix, Eigen::MatrixXd& M);

    /**
   * @brief parseTargetType
   * @param node
   * @param type
   * @return
   */
    bool parseTargetType(const YAML::Node& node, target_t& type);

    /**
   * @brief loadYamlFile
   * @param file
   * @param Q
   * @param P
   * @param R
   * @param type
   * @return
   */
    bool loadYamlFile(const std::string& file, Eigen::MatrixXd& Q, Eigen::MatrixXd& R, Eigen::MatrixXd& P, target_t& type);

    /**
   * @brief targets_ List of Targets
   */
    targets_map_t targets_;

    /**
   * @brief target_lock_ Used to prevent concurrent access to Target vectors
   */
    std::mutex target_lock_;

    /**
   * @brief sphere_origin_ Sphere origin
   */
    Eigen::Vector3d sphere_origin_;

    /**
   * @brief sphere_radius_ Sphere radius
   */
    double sphere_radius_;

    /**
   * @brief default_Q_
   */
    Eigen::MatrixXd default_Q_;

    /**
   * @brief default_P_
   */
    Eigen::MatrixXd default_P_;

    /**
   * @brief default_R_
   */
    Eigen::MatrixXd default_R_;

    /**
   * @brief default_type_
   */
    target_t default_type_;

    /**
   * @brief default_values_loaded_
   */
    bool default_values_loaded_;

};

#endif
