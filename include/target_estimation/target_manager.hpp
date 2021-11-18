/**
*
*/

#ifndef TARGET_MANAGER_HPP
#define TARGET_MANAGER_HPP

#include "target_estimation/target_interface.hpp"
#include <yaml-cpp/yaml.h>

// ----------------------------
/**
 * @brief The TargetManager class
 */
class TargetManager {

public:

    typedef std::shared_ptr<TargetManager> Ptr;

    typedef std::map<unsigned int,TargetInterface::Ptr> targets_map_t;

    enum target_t {ANGULAR_RATES=0,ANGULAR_VELOCITIES,PROJECTILE,UNIFORM_ACCELERATION,UNIFORM_VELOCITY};

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
   * @param t0    Initial time
   * @param p0    Initial estimation of the target's position
   *              Note: we assume p0 = [x y z qx qy qz qw] i.e. size(p0) = 7
   * @param v0    Initial estimation of the target's velocity
   *              Note: we assume v0 = [\dot{x} \dot{y} \dot{z} \omega_x \omega_y \omega_z] i.e. size(v0) = 6
   * @param a0    Initial estimation of the target's acceleration
   *              Note: we assume v0 = [\ddot{x} \ddot{y} \ddot{z} \dot{\omega}_x \dot{\omega}_y \dot{\omega}_z] i.e. size(a0) = 6
   */
    void init(const unsigned int& id, const double& dt0, const double& t0,
              const Eigen::Vector7d& p0, const Eigen::Vector6d& v0 = Eigen::Vector6d::Zero(), const Eigen::Vector6d& a0 = Eigen::Vector6d::Zero());

    /**
   * @brief init  Initialize a new target
   * @param type  Kalman Filter type
   * @param id    Target's id
   * @param dt0   Estimated initial sample time
   * @param t0    Initial time
   * @param Q     Process noise covariance
   * @param R     Measurement noise covariance
   * @param P0    Initial estimation of the error covariance matrix
   * @param p0    Initial estimation of the target's position
   *              Note: we assume p0 = [x y z qx qy qz qw] i.e. size(p0) = 7
   * @param v0    Initial estimation of the target's velocity
   *              Note: we assume v0 = [\dot{x} \dot{y} \dot{z} \omega_x \omega_y \omega_z] i.e. size(v0) = 6
   * @param a0    Initial estimation of the target's acceleration
   *              Note: we assume v0 = [\ddot{x} \ddot{y} \ddot{z} \dot{\omega}_x \dot{\omega}_y \dot{\omega}_z] i.e. size(a0) = 6
   */
    void init(const target_t& type, const unsigned int& id, const double& dt0, const double& t0,
              const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, const Eigen::MatrixXd& P0,
              const Eigen::Vector7d& p0, const Eigen::Vector6d &v0 = Eigen::Vector6d::Zero(), const Eigen::Vector6d &a0 = Eigen::Vector6d::Zero());

    /**
   * @brief init  Initialize a new target
   * @param file  YAML file path with the covariance matrices and target type
   * @param id    Target's id
   * @param dt0   Estimated initial sample time
   * @param t0    Initial time
   * @param p0    Initial estimation of the target's position
   *              Note: we assume p0 = [x y z qx qy qz qw] i.e. size(p0) = 7
   * @param v0    Initial estimation of the target's velocity
   *              Note: we assume v0 = [\dot{x} \dot{y} \dot{z} \omega_x \omega_y \omega_z] i.e. size(v0) = 6
   * @param a0    Initial estimation of the target's acceleration
   *              Note: we assume v0 = [\ddot{x} \ddot{y} \ddot{z} \dot{\omega}_x \dot{\omega}_y \dot{\omega}_z] i.e. size(a0) = 6
   */
    void init(const std::string& file, const unsigned int& id, const double& dt0, const double& t0,
              const Eigen::Vector7d& p0, const Eigen::Vector6d& v0 = Eigen::Vector6d::Zero(), const Eigen::Vector6d& a0 = Eigen::Vector6d::Zero());
    /**
   * @brief update Add a measurement to the given Target ID. Perform an update step with the filters.
   * @param id
   * @param dt
   * @param meas
   * @return false if the selected target does not exist
   */
    bool update(const unsigned int& id, const double& dt, const Eigen::Vector7d& meas);

    /**
   * @brief update perform a predict step with the filters.
   * @param id
   * @param dt
   * @return false if the selected target does not exist
   */
    bool update(const unsigned int& id, const double& dt);

    /**
   * @brief update perform a predict step with all the filters.
   * @param dt
   */
    void update(const double& dt);

    /**
   * @brief erase the target id
   * @param id
   * @return false if the selected target does not exist
   */
    bool erase(const unsigned int& id);

    /**
   * @brief getTarget Return a specific target, a nullptr is returned if the selected target does not exist
   * @param id
   * @return target shared pointer
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
    std::vector<unsigned int> getAvailableTargets();

    /**
   * @brief selectTargetType Convert from string to target_t type
   * @param type_str
   * @param type
   * @return false if type_str does not
   */
    bool selectTargetType(const std::string& type_str, target_t& type);

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
   * @param yaml node
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
