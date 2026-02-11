/**
 * @file robotKinDynamicModel.h
 * @brief Abstract base class for robot kinematics and dynamics models
 * @author Yue Shijie, Huang Yanwei
 * @version 4.0
 * @date 2025-12
 * 
 * @copyright MIT License
 * 
 * Defines the interface for robot models used by the TOPP solver system.
 * Derived classes must implement kinematics (FK, IK), dynamics (M, C, G),
 * friction models, and motion limits.
 * 
 * @section interface_design Interface Design
 * 
 * The interface is designed around the standard robot dynamics equation:
 * @code
 * τ = M(q)q̈ + C(q,q̇)q̇ + G(q) + Fc·sign(q̇) + Fv·q̇
 * @endcode
 * 
 * Where:
 * - M(q): Mass/inertia matrix [n × n]
 * - C(q,q̇): Coriolis/centrifugal matrix [n × n]
 * - G(q): Gravity vector [n]
 * - Fc: Coulomb friction coefficients [n]
 * - Fv: Viscous friction coefficients [n]
 * 
 * @section implementations Implementations
 * 
 * - **elfinKinDynamicModel**: Elfin 6-DOF industrial robot
 * - Future: UR5, KUKA, etc.
 * 
 * @see elfinKinDynamicModel For a complete implementation
 * @see ManagerHandle For accessing robot model through proxy pattern
 */

#ifndef ROBOT_KIN_DYNAMIC_MODEL_H
#define ROBOT_KIN_DYNAMIC_MODEL_H

#include <Eigen/Dense>
#include <tuple>
#include <iostream>
#include <string>
#include <memory>
#include <vector>

/**
 * @brief Abstract base class for robot kinematics and dynamics
 * 
 * Provides the complete interface required by the TOPP solver:
 * - Forward/Inverse Kinematics
 * - Dynamics matrices (M, C, G)
 * - Friction coefficients
 * - Motion constraint limits
 * 
 * @par Design Pattern
 * - **Strategy Pattern**: Different robot implementations
 * - **Non-copyable**: Prevents slicing issues with polymorphism
 * 
 * @par Thread Safety
 * Implementations should be stateless or thread-safe for const methods.
 * 
 * @invariant dof_ >= 1
 * @invariant All returned vectors have dimension dof_
 */
class robotKinDynamicModel {
public:
    /**
     * @brief Construct robot model with name and DOF
     * @param name Robot identifier (e.g., "Elfin5", "UR5")
     * @param dof Degrees of freedom (number of joints)
     */
    robotKinDynamicModel(const std::string& name, int dof) : name_(name), dof_(dof) {}
    
    virtual ~robotKinDynamicModel() = default;
    
    // Disable copy/move to prevent slicing
    robotKinDynamicModel(const robotKinDynamicModel&) = delete;
    robotKinDynamicModel& operator=(const robotKinDynamicModel&) = delete;

    // ════════════════════════════════════════════════════════════════
    // Basic Properties
    // ════════════════════════════════════════════════════════════════
    
    /** @brief Get robot name/identifier */
    [[nodiscard]] const std::string& getName() const { return name_; }
    
    /** @brief Get degrees of freedom (number of joints) */
    [[nodiscard]] int getDOF() const { return dof_; }

    // ════════════════════════════════════════════════════════════════
    // Kinematics Interface
    // ════════════════════════════════════════════════════════════════
    
    /**
     * @brief Compute forward kinematics (base to end-effector)
     * @param q Joint configuration [rad], size = dof_
     * @return 4×4 homogeneous transformation matrix
     */
    [[nodiscard]] virtual Eigen::Matrix4d forwardKinematics(const Eigen::VectorXd& q) const = 0;
    
    /**
     * @brief Compute inverse kinematics
     * @param T Target end-effector pose (4×4 homogeneous matrix)
     * @param q_seed Reference/seed configuration for solution selection
     * @param[out] q_solution Resulting joint configuration if successful
     * @return true if valid solution found, false otherwise
     */
    [[nodiscard]] virtual bool inverseKinematics(const Eigen::Matrix4d& T, 
                                                 const Eigen::VectorXd& q_seed,
                                                 Eigen::VectorXd& q_solution) const = 0;

    // ════════════════════════════════════════════════════════════════
    // Dynamics Interface
    // ════════════════════════════════════════════════════════════════
    
    /**
     * @brief Compute inverse dynamics (joint torques)
     * 
     * τ = M(q)q̈ + C(q,q̇)q̇ + G(q)
     * 
     * @param q Joint positions [rad]
     * @param q_dot Joint velocities [rad/s]
     * @param q_ddot Joint accelerations [rad/s²]
     * @return Joint torques [N·m]
     * 
     * @note Default implementation uses getMCGMatrix()
     */
    [[nodiscard]] virtual Eigen::VectorXd inverseDynamics(const Eigen::VectorXd& q,
                                                          const Eigen::VectorXd& q_dot,
                                                          const Eigen::VectorXd& q_ddot) const;

    /**
     * @brief Compute dynamics matrices M, C, G
     * 
     * @param q_i Joint positions [rad]
     * @param dq_i Joint velocities [rad/s] (used for C matrix)
     * @return Tuple of (M, C, G):
     *   - M: Mass matrix [n × n]
     *   - C: Coriolis matrix [n × n]
     *   - G: Gravity vector [n]
     */
    [[nodiscard]] virtual std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::VectorXd>
    getMCGMatrix(const Eigen::VectorXd& q_i, const Eigen::VectorXd& dq_i) const;

    /**
     * @brief Get friction coefficients for TOPP constraint formulation
     * 
     * Used in TOPP to include friction in dynamics:
     * τ = M·q̈ + C·q̇ + G + Fc·sign(q̇) + Fv·q̇
     * 
     * @param coulomb If true, return Coulomb friction (Fc)
     *                If false, return viscous friction (Fv)
     * @return Friction coefficients [dof_]
     * 
     * @par MATLAB Alignment
     * Aligned with MATLAB getFrictionCoeffs() function
     */
    [[nodiscard]] virtual Eigen::VectorXd getFrictionCoefficients(bool coulomb) const = 0;

    // ════════════════════════════════════════════════════════════════
    // Motion Limits
    // ════════════════════════════════════════════════════════════════
    
    /** @brief Get per-joint velocity limits [rad/s] */
    [[nodiscard]] virtual Eigen::VectorXd getDefaultVelocityLimits() const = 0;
    
    /** @brief Get per-joint acceleration limits [rad/s²] */
    [[nodiscard]] virtual Eigen::VectorXd getDefaultAccelerationLimits() const = 0;
    
    /** @brief Get per-joint jerk limits [rad/s³] */
    [[nodiscard]] virtual Eigen::VectorXd getDefaultJerkLimits() const = 0;
    
    /** @brief Get per-joint torque limits [N·m] */
    [[nodiscard]] virtual Eigen::VectorXd getDefaultTorqueLimits() const = 0;
    
    /** @brief Print model info to stdout */
    void printInfo() const {
        std::cout << "========== Robot Model: " << name_ << " ==========" << std::endl;
        std::cout << "DOF: " << dof_ << std::endl;
        std::cout << "==========================================" << std::endl;
    }

protected:
    std::string name_;  ///< Robot identifier
    int dof_;           ///< Degrees of freedom
};

using robotKinDynamicModelPtr = std::shared_ptr<robotKinDynamicModel>;

#endif // ROBOT_KIN_DYNAMIC_MODEL_H