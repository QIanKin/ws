#pragma once

/**
 * @file types.hpp
 * @brief MOCHA trajectory optimization core data structures
 * @details Contains parameter structs, optimization data wrappers, and trajectory results.
 *          No ROS dependencies -- only Eigen and standard C++.
 */

#include <vector>
#include <string>
#include <functional>
#include <Eigen/Core>

namespace mocha {

// ==================== ESDF query callback ====================

/**
 * @brief ESDF query function type.
 * @param pos   World position (x, y)
 * @param dist  Output: signed distance (positive = free, negative = inside obstacle)
 * @param grad  Output: normalized gradient (points away from nearest obstacle)
 * @return true if query succeeded
 */
using EsdfQueryFunc = std::function<bool(const Eigen::Vector2d& pos, double& dist, Eigen::Vector2d& grad)>;

/**
 * @brief Time-indexed risk query function type.
 * @param pos   World position (x, y)
 * @param t_rel Time relative to plan start (s)
 * @param risk  Output: occupancy/risk probability in [0, 1]
 * @param grad  Output: spatial risk gradient
 * @return true if query succeeded
 */
using RiskQueryFunc = std::function<bool(const Eigen::Vector2d& pos, double t_rel, double& risk, Eigen::Vector2d& grad)>;

/**
 * @brief Time-indexed corridor signed-distance query.
 * @param pos   World position (x, y)
 * @param t_rel Time relative to plan start (s)
 * @param dist  Output: signed distance to corridor boundary (positive = inside)
 * @param grad  Output: spatial gradient of signed distance
 * @return true if query succeeded
 */
using CorridorQueryFunc = std::function<bool(const Eigen::Vector2d& pos, double t_rel, double& dist, Eigen::Vector2d& grad)>;

// ==================== Dynamic obstacle model ====================

/**
 * @struct DynamicObstacle
 * @brief Linear (constant-velocity) dynamic obstacle for trajectory optimization.
 *
 * Position model:  p_obs(t) = position + velocity * (t - t_ref)
 * where t is absolute wall-clock time.
 */
struct DynamicObstacle
{
    Eigen::Vector2d position;           ///< obstacle centre at reference time (m)
    Eigen::Vector2d velocity;           ///< constant velocity (m/s)
    double t_ref = 0.0;                 ///< reference time for position (s)
    double radius = 0.3;                ///< obstacle radius (m)
};

// ==================== MCO parameters ====================

/**
 * @struct McoParameters
 * @brief All inputs required by the MCO trajectory optimiser
 */
struct McoParameters
{
    // ---- dynamics / geometry ----
    double v_max = 2.5;               ///< maximum speed (m/s)
    double a_max = 5.0;               ///< maximum acceleration (m/s^2)
    double drone_radius = 0.20;       ///< robot radius (m)

    // ---- polynomial trajectory constants ----
    const int dims = 2;
    const int s = 3;
    const int n_order = 2 * s - 1;       // = 5
    const int n_coeffs = n_order + 1;     // = 6

    // ---- optimisation weights ----
    double w_energy = 1.0;
    double w_time = 10.0;
    double w_feasibility = 5000.0;
    double w_obstacle = 5000.0;
    double w_distance = 100.0;           ///< waypoint distance regularisation to suppress unnecessary bending

    // ---- ESDF-based collision avoidance ----
    EsdfQueryFunc esdf_query;          ///< ESDF query function (required)
    double esdf_safety_margin{0.1};    ///< additional safety margin beyond drone_radius (m)
    double w_obstacle_soft{2000.0};    ///< soft-zone penalty weight (provides far-range gradient)
    double esdf_soft_margin{0.5};      ///< soft zone extends this far beyond d_margin (m)
    double obstacle_demarcation{0.1};  ///< cubic-to-quadratic switchover depth in hard zone (m)

    // ---- DSP/risk-map avoidance ----
    RiskQueryFunc risk_query;          ///< time-indexed risk query (optional)
    double w_risk{0.0};                ///< risk penalty weight
    double risk_threshold{0.0};        ///< risk values <= threshold are free
    double risk_power{2.0};            ///< risk penalty exponent, clamped to >=1
    double risk_time_gradient_eps{0.05}; ///< finite-difference time step for dRisk/dt

    // ---- risk-corridor soft constraint ----
    CorridorQueryFunc corridor_query;   ///< time-indexed single-topology corridor query (optional)
    double w_corridor{0.0};             ///< corridor soft-barrier penalty weight
    double corridor_margin{0.0};        ///< desired minimum signed distance inside corridor (m)
    double corridor_power{3.0};         ///< corridor penalty exponent, clamped to >=1

    // ---- dynamic obstacle avoidance ----
    std::vector<DynamicObstacle> dynamic_obstacles;   ///< moving obstacles (may be empty)
    double w_dynamic_obs = 5000.0;                    ///< dynamic obstacle penalty weight
    double dynamic_safety_margin = 0.15;              ///< additional margin beyond drone_radius + obs_radius (m)
    double t_plan_start = 0.0;                        ///< absolute time at which this plan starts (s)

    // ---- penalty sampling ----
    int kappa = 10;

    // ---- boundary conditions ----
    Eigen::Vector2d start_waypoint;
    Eigen::Vector2d end_waypoint;
    Eigen::Vector2d start_vel = Eigen::Vector2d::Zero();
    Eigen::Vector2d start_acc = Eigen::Vector2d::Zero();
    Eigen::Vector2d end_vel   = Eigen::Vector2d::Zero();
    Eigen::Vector2d end_acc   = Eigen::Vector2d::Zero();

    // ---- segment layout ----
    int n_segments;
    std::vector<double> initial_segment_times;
    double initial_time_speed_ratio{0.65};  ///< warm-start segment time = seg_length / max(0.1, v_max * ratio)

    // ---- solver configuration ----
    int lbfgs_max_iterations{80};           ///< L-BFGS iteration cap

    // ---- motion camouflage ----
    bool use_vmc = true;                  ///< true=VMC降维(v参数), false=纯MINCO(直接优化xy)
    Eigen::Vector2d ref_point = Eigen::Vector2d::Zero();
    std::vector<Eigen::Vector2d> prey_points;
};

// ==================== Trajectory result ====================

/**
 * @struct McoTrajectory
 * @brief Output of the MCO optimiser
 */
struct McoTrajectory
{
    Eigen::MatrixXd coeffs;    ///< polynomial coefficients  (n_coeffs*n_segments) x dims
    Eigen::VectorXd T;         ///< per-segment durations     n_segments x 1
    double total_duration;     ///< sum of T
    double final_cost{0.0};    ///< cost at convergence

    bool isValid() const {
        return T.size() > 0 && coeffs.size() > 0;
    }
};

} // namespace mocha
