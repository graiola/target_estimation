/**
*/

#ifndef KALMAN_HPP
#define KALMAN_HPP

#include <Eigen/Dense>
#include <memory>

/**
 * @brief The KalmanFilterInterface class
 * The Kalman filter makes a prediction, takes a measurement, and then forms a new estimate somewhere between the two.
 * Reference https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python
 */
class KalmanFilterInterface {

public:

    typedef std::shared_ptr<KalmanFilterInterface> Ptr;

    /**
  * Create a blank estimator.
  */
    KalmanFilterInterface();

    /**
  * Destructor.
  */
    virtual ~KalmanFilterInterface() {}

    /**
  * Initialize the filter with initial states as zero.
  */
    void init();

    /**
  * Initialize the filter with a guess for initial states.
  */
    void init(const Eigen::VectorXd& x0);

    /**
  * Update the estimated state based on measured values.
  */
   virtual void update(const Eigen::VectorXd& y);

    /**
  * Update the estimated state based on predicted values.
  */
   virtual void update();

   /**
    * Return the current state.
    */
   Eigen::VectorXd& getState() {return x_hat_;}

   /**
    * Return the Q matrix.
    */
   Eigen::MatrixXd& getQ() {return Q_;}

   /**
    * Return the R matrix.
    */
   Eigen::MatrixXd& getR() {return R_;}

   /**
    * Return the P matrix.
    */
   Eigen::MatrixXd& getP() {return P_;}

   /**
    * Return the P0 matrix (initial value for P).
    */
   Eigen::MatrixXd& getP0() {return P0_;}


 protected:

    /**
     * @brief predict
     */
    virtual void predict() = 0;

    /**
     * @brief estimate
     */
    virtual void estimate(const Eigen::VectorXd& y) = 0;

    /**
     * @brief Matrices for computation
     */
    Eigen::MatrixXd A_, C_, Q_, R_, P_, K_, P0_;

    /**
     * @brief System dimensions
     */
    int m_, n_;

    /**
     * @brief Is the filter initialized?
     */
    bool initialized_;

    /**
     * @brief n-size identity
     */
    Eigen::MatrixXd I_;

    /**
     * @brief Estimated states
     */
    Eigen::VectorXd x_hat_, x_hat_new_;
};

class LinearKalmanFilter : public KalmanFilterInterface
{

public:

    typedef std::shared_ptr<LinearKalmanFilter> Ptr;

    /**
    * Create a linear Kalman filter with the specified matrices.
    *   A - System dynamics matrix
    *   C - Output matrix
    *   Q - Process noise covariance
    *   R - Measurement noise covariance
    *   P - Estimate error covariance
    */
    LinearKalmanFilter(
            const Eigen::MatrixXd& A,
            const Eigen::MatrixXd& C,
            const Eigen::MatrixXd& Q,
            const Eigen::MatrixXd& R,
            const Eigen::MatrixXd& P
            );


    /**
  * Destructor.
  */
    virtual ~LinearKalmanFilter() {}

    /**
  * Update the estimated state based on predicted values using an updated dynamics matrix
  */
   virtual void update(const Eigen::MatrixXd& A);

    /**
  * Update the estimated state based on measured values using an updated dynamics matrix
  */
   virtual void update(const Eigen::VectorXd& y, const Eigen::MatrixXd& A);

private:

    /**
     * @brief predict
     */
    virtual void predict() override;

    /**
     * @brief estimate
     */
    virtual void estimate(const Eigen::VectorXd& y) override;

protected:

    /**
  * Create a blank estimator.
  */
    LinearKalmanFilter();

};

class ExtendedKalmanFilter : public LinearKalmanFilter
{

public:

    typedef std::shared_ptr<ExtendedKalmanFilter> Ptr;

    typedef std::function<Eigen::VectorXd (const Eigen::VectorXd&)> nonliner_function_t;

    /**
    * Create an extended Kalman filter for non linear systems.
    * We define the following:
    *   f - Non linear system dynamics
    *   h - Non linear system output
    *   A - System dynamics matrix (linearized from f)
    *   C - Output matrix (linearized from h)
    *   Q - Process noise covariance
    *   R - Measurement noise covariance
    *   P - Estimate error covariance
    */
    ExtendedKalmanFilter(
            nonliner_function_t f,
            nonliner_function_t h,
            const Eigen::MatrixXd& A,
            const Eigen::MatrixXd& C,
            const Eigen::MatrixXd& Q,
            const Eigen::MatrixXd& R,
            const Eigen::MatrixXd& P
            );

    /**
  * Destructor.
  */
    virtual ~ExtendedKalmanFilter() {}

    /**
  * Update the estimated state based on predicted values using an updated transition function
  */
   void update(nonliner_function_t f, const Eigen::MatrixXd& A);

    /**
  * Update the estimated state based on measured values using an updated transition function
  */
   void update(const Eigen::VectorXd& y, nonliner_function_t f, const Eigen::MatrixXd& A);

   /**
 * Update the estimated state based on measured values.
 */
  virtual void update(const Eigen::VectorXd& y);

   /**
 * Update the estimated state based on predicted values.
 */
  virtual void update();

private:

    /**
     * @brief predict
     */
    virtual void predict() override;

    /**
     * @brief estimate
     */
    virtual void estimate(const Eigen::VectorXd& y) override;

protected:

   /**
 * Create a blank estimator.
 */
   ExtendedKalmanFilter();

   nonliner_function_t f_;
   nonliner_function_t h_;

};

#endif
