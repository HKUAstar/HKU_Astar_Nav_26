// SPDX-License-Identifier: BSD-2-Clause
//
// ============================================================================
// ANNOTATED VERSION OF graph_slam.cpp
// ============================================================================
// This file implements the GraphSLAM class, which is a wrapper around the g2o
// (General Graph Optimization) library. It provides a clean API for building
// and optimizing a 3D pose graph used in LiDAR SLAM.
//
// In graph-based SLAM:
//   - NODES (vertices) represent robot poses or environmental features
//     (planes, points) at different moments in time.
//   - EDGES represent constraints (measurements) between nodes, such as
//     "the robot moved X meters between pose A and pose B" (odometry),
//     or "this pose is at GPS coordinate Y" (prior).
//   - OPTIMIZATION adjusts all node positions simultaneously to minimize
//     the total error across all edge constraints.
// ============================================================================

// --- Include the class declaration (header) ---
#include <hdl_graph_slam/graph_slam.hpp>

// --- Standard / Boost utilities ---
#include <boost/format.hpp>       // boost::format for formatted console output

// --- g2o core framework ---
#include <g2o/stuff/macros.h>                         // g2o utility macros
#include <g2o/core/factory.h>                         // Factory pattern for creating g2o types at runtime
#include <g2o/core/block_solver.h>                    // Block matrix solver (used internally by g2o)
#include <g2o/core/linear_solver.h>                   // Linear algebra solver interface
#include <g2o/core/sparse_optimizer.h>                // Main g2o class: a sparse graph optimizer
#include <g2o/core/robust_kernel_factory.h>           // Factory for robust kernels (Huber, Cauchy, etc.)
#include <g2o/core/optimization_algorithm.h>          // Base class for optimization algorithms (GN, LM)
#include <g2o/core/optimization_algorithm_factory.h>  // Factory to select optimization algorithm by name
#include <g2o/solvers/pcg/linear_solver_pcg.h>        // Preconditioned Conjugate Gradient linear solver

// --- g2o built-in 3D SLAM types ---
#include <g2o/types/slam3d/types_slam3d.h>            // SE3 vertices & edges (VertexSE3, EdgeSE3, etc.)
#include <g2o/types/slam3d/edge_se3_pointxyz.h>       // Edge between a pose (SE3) and a 3D point
#include <g2o/types/slam3d_addons/types_slam3d_addons.h> // Additional types: planes, etc.

// --- Custom g2o edge types defined by hdl_graph_slam ---
// These are specialized constraint types not available in vanilla g2o.
#include <g2o/edge_se3_plane.hpp>           // Constraint: a pose observes a plane (floor detection)
#include <g2o/edge_se3_priorxy.hpp>         // Prior constraint on a pose's XY position (2D GPS)
#include <g2o/edge_se3_priorxyz.hpp>        // Prior constraint on a pose's XYZ position (3D GPS)
#include <g2o/edge_se3_priorvec.hpp>        // Prior constraint on a direction vector (IMU gravity)
#include <g2o/edge_se3_priorquat.hpp>       // Prior constraint on orientation quaternion (IMU compass)
#include <g2o/edge_plane_prior.hpp>         // Prior on a plane's normal direction or distance
#include <g2o/edge_plane_identity.hpp>      // Constraint: two planes should be the same plane
#include <g2o/edge_plane_parallel.hpp>      // Constraint: two planes should be parallel
#include <g2o/robust_kernel_io.hpp>         // Save/load robust kernels to/from files


// ============================================================================
// REGISTER OPTIMIZATION LIBRARIES
// ============================================================================
// These macros tell g2o to link in specific sparse linear solvers.
// When you select a solver like "lm_var_cholmod", g2o needs the cholmod
// library to actually be registered. These macros do that registration.
//
// PCG     = Preconditioned Conjugate Gradient (iterative, no external deps)
// CHOLMOD = Sparse Cholesky factorization (fast, but GPL licensed)
// CSPARSE = Sparse direct solver (LGPL if dynamically linked)
// ============================================================================
G2O_USE_OPTIMIZATION_LIBRARY(pcg)
G2O_USE_OPTIMIZATION_LIBRARY(cholmod)  // WARNING: cholmod brings GPL dependency
G2O_USE_OPTIMIZATION_LIBRARY(csparse)  // WARNING: csparse brings LGPL unless dynamically linked


// ============================================================================
// REGISTER CUSTOM EDGE TYPES WITH g2o's FACTORY
// ============================================================================
// g2o uses a factory pattern so it can serialize/deserialize graph files.
// Each custom edge type needs a unique string identifier (e.g. "EDGE_SE3_PLANE")
// and must be registered so g2o can reconstruct them when loading a saved graph.
// ============================================================================
namespace g2o {
G2O_REGISTER_TYPE(EDGE_SE3_PLANE, EdgeSE3Plane)                        // pose-to-plane constraint
G2O_REGISTER_TYPE(EDGE_SE3_PRIORXY, EdgeSE3PriorXY)                    // 2D GPS position prior
G2O_REGISTER_TYPE(EDGE_SE3_PRIORXYZ, EdgeSE3PriorXYZ)                  // 3D GPS position prior
G2O_REGISTER_TYPE(EDGE_SE3_PRIORVEC, EdgeSE3PriorVec)                  // direction vector prior (IMU gravity)
G2O_REGISTER_TYPE(EDGE_SE3_PRIORQUAT, EdgeSE3PriorQuat)                // orientation quaternion prior (IMU compass)
G2O_REGISTER_TYPE(EDGE_PLANE_PRIOR_NORMAL, EdgePlanePriorNormal)        // prior on plane normal direction
G2O_REGISTER_TYPE(EDGE_PLANE_PRIOR_DISTANCE, EdgePlanePriorDistance)    // prior on plane distance from origin
G2O_REGISTER_TYPE(EDGE_PLANE_IDENTITY, EdgePlaneIdentity)               // two planes are the same
G2O_REGISTER_TYPE(EDGE_PLANE_PARALLEL, EdgePlaneParallel)               // two planes are parallel
G2O_REGISTER_TYPE(EDGE_PLANE_PAERPENDICULAR, EdgePlanePerpendicular)    // two planes are perpendicular
// NOTE: "PAERPENDICULAR" is a typo in the original code (should be PERPENDICULAR)
// but it must stay this way for file format compatibility.
}  // namespace g2o


namespace hdl_graph_slam {

// ============================================================================
// CONSTRUCTOR
// ============================================================================
// Creates the g2o sparse optimizer and configures it with the requested solver.
//
// solver_type examples:
//   "lm_var"         = Levenberg-Marquardt with variable block sizes
//   "lm_var_cholmod" = LM using the Cholmod sparse Cholesky solver (fastest)
//   "gn_var"         = Gauss-Newton with variable block sizes
//   "gn_var_cholmod" = GN using Cholmod
//
// The naming convention is: <algorithm>_<block_structure>_<linear_solver>
//   algorithm:       gn = Gauss-Newton, lm = Levenberg-Marquardt
//   block_structure: var = variable, fix6_3 = fixed 6x3
//   linear_solver:   cholmod, csparse, pcg (omitted = default)
// ============================================================================
GraphSLAM::GraphSLAM(const std::string& solver_type) {
  // Create an empty sparse optimizer (the graph container + optimizer).
  graph.reset(new g2o::SparseOptimizer());

  // Cast the base HyperGraph pointer to a SparseOptimizer to access
  // optimization-specific methods.
  g2o::SparseOptimizer* graph = dynamic_cast<g2o::SparseOptimizer*>(this->graph.get());

  std::cout << "construct solver: " << solver_type << std::endl;

  // Use g2o's factory to construct the right solver by name string.
  // The factory knows about all registered optimization libraries (pcg, cholmod, csparse)
  // and all algorithm types (Gauss-Newton, Levenberg-Marquardt, etc.).
  g2o::OptimizationAlgorithmFactory* solver_factory = g2o::OptimizationAlgorithmFactory::instance();
  g2o::OptimizationAlgorithmProperty solver_property;
  g2o::OptimizationAlgorithm* solver = solver_factory->construct(solver_type, solver_property);

  // Assign the solver to the graph optimizer.
  graph->setAlgorithm(solver);

  // Safety check: if the solver failed to initialize, print available solvers and wait.
  if(!graph->solver()) {
    std::cerr << std::endl;
    std::cerr << "error : failed to allocate solver!!" << std::endl;
    solver_factory->listSolvers(std::cerr);   // Print all available solver names
    std::cerr << "-------------" << std::endl;
    std::cin.ignore(1);                        // Pause so the user can read the error
    return;
  }
  std::cout << "done" << std::endl;

  // Get the singleton robust kernel factory. This will be used later
  // when adding robust kernels (Huber, Cauchy, etc.) to edges.
  robust_kernel_factory = g2o::RobustKernelFactory::instance();
}

// ============================================================================
// DESTRUCTOR
// ============================================================================
// Releases the g2o graph and all vertices/edges it owns.
// ============================================================================
GraphSLAM::~GraphSLAM() {
  graph.reset();  // unique_ptr reset -> deletes the SparseOptimizer
}

// ============================================================================
// SET SOLVER (runtime reconfiguration)
// ============================================================================
// Allows changing the solver type after construction. Useful if you want to
// switch between Gauss-Newton and Levenberg-Marquardt at runtime.
// ============================================================================
void GraphSLAM::set_solver(const std::string& solver_type) {
  g2o::SparseOptimizer* graph = dynamic_cast<g2o::SparseOptimizer*>(this->graph.get());

  std::cout << "construct solver: " << solver_type << std::endl;
  g2o::OptimizationAlgorithmFactory* solver_factory = g2o::OptimizationAlgorithmFactory::instance();
  g2o::OptimizationAlgorithmProperty solver_property;
  g2o::OptimizationAlgorithm* solver = solver_factory->construct(solver_type, solver_property);
  graph->setAlgorithm(solver);

  if(!graph->solver()) {
    std::cerr << std::endl;
    std::cerr << "error : failed to allocate solver!!" << std::endl;
    solver_factory->listSolvers(std::cerr);
    std::cerr << "-------------" << std::endl;
    std::cin.ignore(1);
    return;
  }
  std::cout << "done" << std::endl;
}

// ============================================================================
// GRAPH QUERY METHODS
// ============================================================================
int GraphSLAM::num_vertices() const {
  return graph->vertices().size();  // Total number of nodes in the graph
}
int GraphSLAM::num_edges() const {
  return graph->edges().size();     // Total number of edges (constraints) in the graph
}


// ============================================================================
// ============================================================================
//                        NODE (VERTEX) CREATION
// ============================================================================
// ============================================================================
// Nodes represent estimated quantities in the graph:
//   - SE3 nodes: 6-DOF robot poses (position + orientation)
//   - Plane nodes: planar surfaces (4D: normal_x, normal_y, normal_z, distance)
//   - PointXYZ nodes: 3D point landmarks
//
// Each node gets a unique integer ID (auto-incrementing based on graph size).
// ============================================================================

// --- Add a 6-DOF robot pose node ---
// Isometry3d encodes both rotation (3x3 matrix) and translation (3x1 vector).
// This is the most common node type: every keyframe in the SLAM adds one.
g2o::VertexSE3* GraphSLAM::add_se3_node(const Eigen::Isometry3d& pose) {
  g2o::VertexSE3* vertex(new g2o::VertexSE3());
  vertex->setId(static_cast<int>(graph->vertices().size()));  // Auto-increment ID
  vertex->setEstimate(pose);                                  // Set initial pose guess
  graph->addVertex(vertex);                                   // Add to the graph
  return vertex;
}

// --- Add a plane node ---
// Plane is represented as (a, b, c, d) where ax + by + cz + d = 0.
// Used by floor detection to represent detected floor planes.
g2o::VertexPlane* GraphSLAM::add_plane_node(const Eigen::Vector4d& plane_coeffs) {
  g2o::VertexPlane* vertex(new g2o::VertexPlane());
  vertex->setId(static_cast<int>(graph->vertices().size()));
  vertex->setEstimate(plane_coeffs);
  graph->addVertex(vertex);
  return vertex;
}

// --- Add a 3D point landmark node ---
// Used if you want to track specific point features (not commonly used
// in the default hdl_graph_slam pipeline, but the API supports it).
g2o::VertexPointXYZ* GraphSLAM::add_point_xyz_node(const Eigen::Vector3d& xyz) {
  g2o::VertexPointXYZ* vertex(new g2o::VertexPointXYZ());
  vertex->setId(static_cast<int>(graph->vertices().size()));
  vertex->setEstimate(xyz);
  graph->addVertex(vertex);
  return vertex;
}


// ============================================================================
// ============================================================================
//                          EDGE (CONSTRAINT) CREATION
// ============================================================================
// ============================================================================
// Edges encode measurements/constraints between nodes. Each edge stores:
//   1. The measurement (e.g., relative pose between two keyframes)
//   2. An information matrix (inverse covariance) encoding measurement confidence
//   3. Pointers to the connected vertices
//
// During optimization, g2o adjusts node values to minimize the weighted sum
// of squared errors across all edges:
//   E_total = Σ (error_i^T * Ω_i * error_i)
// where Ω_i is the information matrix and error_i = measurement - prediction.
// ============================================================================

// --- SE3-to-SE3 edge (ODOMETRY / LOOP CLOSURE) ---
// This is THE most important edge type. It encodes:
//   "The relative transformation from pose v1 to pose v2 is <relative_pose>"
// Used for: consecutive keyframe odometry, loop closure constraints.
// Information matrix is 6x6 (3 translation + 3 rotation DOF).
g2o::EdgeSE3* GraphSLAM::add_se3_edge(g2o::VertexSE3* v1, g2o::VertexSE3* v2, const Eigen::Isometry3d& relative_pose, const Eigen::MatrixXd& information_matrix) {
  g2o::EdgeSE3* edge(new g2o::EdgeSE3());
  edge->setMeasurement(relative_pose);         // The observed relative transform
  edge->setInformation(information_matrix);     // Confidence (higher = more trusted)
  edge->vertices()[0] = v1;                     // First connected pose
  edge->vertices()[1] = v2;                     // Second connected pose
  graph->addEdge(edge);
  return edge;
}

// --- SE3-to-Plane edge (FLOOR CONSTRAINT) ---
// Constrains: "From pose v_se3, the observed plane has coefficients <plane_coeffs>"
// Used by floor detection: ensures all keyframes agree on the floor plane.
g2o::EdgeSE3Plane* GraphSLAM::add_se3_plane_edge(g2o::VertexSE3* v_se3, g2o::VertexPlane* v_plane, const Eigen::Vector4d& plane_coeffs, const Eigen::MatrixXd& information_matrix) {
  g2o::EdgeSE3Plane* edge(new g2o::EdgeSE3Plane());
  edge->setMeasurement(plane_coeffs);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v_se3;
  edge->vertices()[1] = v_plane;
  graph->addEdge(edge);
  return edge;
}

// --- SE3-to-PointXYZ edge ---
// Constrains: "From pose v_se3, a point landmark was observed at <xyz>"
// Standard in visual SLAM; less common in pure LiDAR SLAM.
g2o::EdgeSE3PointXYZ* GraphSLAM::add_se3_point_xyz_edge(g2o::VertexSE3* v_se3, g2o::VertexPointXYZ* v_xyz, const Eigen::Vector3d& xyz, const Eigen::MatrixXd& information_matrix) {
  g2o::EdgeSE3PointXYZ* edge(new g2o::EdgeSE3PointXYZ());
  edge->setMeasurement(xyz);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v_se3;
  edge->vertices()[1] = v_xyz;
  graph->addEdge(edge);
  return edge;
}

// --- Plane Normal Prior ---
// A unary constraint: "This plane's normal should be <normal>"
// Used to enforce that the floor plane points upward (gravity direction).
g2o::EdgePlanePriorNormal* GraphSLAM::add_plane_normal_prior_edge(g2o::VertexPlane* v, const Eigen::Vector3d& normal, const Eigen::MatrixXd& information_matrix) {
  g2o::EdgePlanePriorNormal* edge(new g2o::EdgePlanePriorNormal());
  edge->setMeasurement(normal);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v;          // Unary edge: only one vertex
  graph->addEdge(edge);
  return edge;
}

// --- Plane Distance Prior ---
// A unary constraint: "This plane's distance from origin should be <distance>"
// Used to fix the floor height (e.g., floor is at z=0).
g2o::EdgePlanePriorDistance* GraphSLAM::add_plane_distance_prior_edge(g2o::VertexPlane* v, double distance, const Eigen::MatrixXd& information_matrix) {
  g2o::EdgePlanePriorDistance* edge(new g2o::EdgePlanePriorDistance());
  edge->setMeasurement(distance);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v;
  graph->addEdge(edge);
  return edge;
}

// --- SE3 Prior XY (2D GPS) ---
// A unary constraint: "This pose's XY position should be <xy>"
// Used when GPS altitude is NaN (treat as 2D constraint only).
// Information matrix is 2x2.
g2o::EdgeSE3PriorXY* GraphSLAM::add_se3_prior_xy_edge(g2o::VertexSE3* v_se3, const Eigen::Vector2d& xy, const Eigen::MatrixXd& information_matrix) {
  g2o::EdgeSE3PriorXY* edge(new g2o::EdgeSE3PriorXY());
  edge->setMeasurement(xy);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v_se3;
  graph->addEdge(edge);
  return edge;
}

// --- SE3 Prior XYZ (3D GPS) ---
// A unary constraint: "This pose's XYZ position should be <xyz>"
// Used when full GPS coordinates (lat, lon, alt) are available.
// Information matrix is 3x3.
g2o::EdgeSE3PriorXYZ* GraphSLAM::add_se3_prior_xyz_edge(g2o::VertexSE3* v_se3, const Eigen::Vector3d& xyz, const Eigen::MatrixXd& information_matrix) {
  g2o::EdgeSE3PriorXYZ* edge(new g2o::EdgeSE3PriorXYZ());
  edge->setMeasurement(xyz);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v_se3;
  graph->addEdge(edge);
  return edge;
}

// --- SE3 Prior Vector (IMU ACCELERATION / GRAVITY) ---
// A unary constraint: "At this pose, the direction vector <direction> should
// align with measurement vector <measurement>"
// Used for IMU gravity alignment: direction=(0,0,1), measurement=gravity_in_sensor_frame.
// The 6D measurement packs <direction, measurement> into a single vector.
// Information matrix is 3x3.
g2o::EdgeSE3PriorVec* GraphSLAM::add_se3_prior_vec_edge(g2o::VertexSE3* v_se3, const Eigen::Vector3d& direction, const Eigen::Vector3d& measurement, const Eigen::MatrixXd& information_matrix) {
  Eigen::Matrix<double, 6, 1> m;
  m.head<3>() = direction;     // First 3 elements: reference direction in world frame
  m.tail<3>() = measurement;   // Last 3 elements: measured direction in sensor frame

  g2o::EdgeSE3PriorVec* edge(new g2o::EdgeSE3PriorVec());
  edge->setMeasurement(m);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v_se3;
  graph->addEdge(edge);
  return edge;
}

// --- SE3 Prior Quaternion (IMU ORIENTATION / COMPASS) ---
// A unary constraint: "This pose's orientation should be <quat>"
// Used for IMU magnetic orientation data.
// Information matrix is 3x3 (rotation has 3 DOF).
g2o::EdgeSE3PriorQuat* GraphSLAM::add_se3_prior_quat_edge(g2o::VertexSE3* v_se3, const Eigen::Quaterniond& quat, const Eigen::MatrixXd& information_matrix) {
  g2o::EdgeSE3PriorQuat* edge(new g2o::EdgeSE3PriorQuat());
  edge->setMeasurement(quat);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v_se3;
  graph->addEdge(edge);
  return edge;
}

// --- Plane-to-Plane edge ---
// Constrains the relationship between two plane nodes.
// General form with 4D measurement and 4x4 information.
g2o::EdgePlane* GraphSLAM::add_plane_edge(g2o::VertexPlane* v_plane1, g2o::VertexPlane* v_plane2, const Eigen::Vector4d& measurement, const Eigen::Matrix4d& information) {
  g2o::EdgePlane* edge(new g2o::EdgePlane());
  edge->setMeasurement(measurement);
  edge->setInformation(information);
  edge->vertices()[0] = v_plane1;
  edge->vertices()[1] = v_plane2;
  graph->addEdge(edge);
  return edge;
}

// --- Plane Identity edge ---
// Constrains: "These two planes are the same physical plane"
// Used in floor detection to merge floor observations across keyframes.
g2o::EdgePlaneIdentity* GraphSLAM::add_plane_identity_edge(g2o::VertexPlane* v_plane1, g2o::VertexPlane* v_plane2, const Eigen::Vector4d& measurement, const Eigen::Matrix4d& information) {
  g2o::EdgePlaneIdentity* edge(new g2o::EdgePlaneIdentity());
  edge->setMeasurement(measurement);
  edge->setInformation(information);
  edge->vertices()[0] = v_plane1;
  edge->vertices()[1] = v_plane2;
  graph->addEdge(edge);
  return edge;
}

// --- Plane Parallel edge ---
// Constrains: "These two planes are parallel (same normal direction)"
// Information matrix is 3x3 (only the normal is constrained, not the distance).
g2o::EdgePlaneParallel* GraphSLAM::add_plane_parallel_edge(g2o::VertexPlane* v_plane1, g2o::VertexPlane* v_plane2, const Eigen::Vector3d& measurement, const Eigen::Matrix3d& information) {
  g2o::EdgePlaneParallel* edge(new g2o::EdgePlaneParallel());
  edge->setMeasurement(measurement);
  edge->setInformation(information);
  edge->vertices()[0] = v_plane1;
  edge->vertices()[1] = v_plane2;
  graph->addEdge(edge);
  return edge;
}

// --- Plane Perpendicular edge ---
// Constrains: "These two planes are perpendicular"
g2o::EdgePlanePerpendicular* GraphSLAM::add_plane_perpendicular_edge(g2o::VertexPlane* v_plane1, g2o::VertexPlane* v_plane2, const Eigen::Vector3d& measurement, const Eigen::MatrixXd& information) {
  g2o::EdgePlanePerpendicular* edge(new g2o::EdgePlanePerpendicular());
  edge->setMeasurement(measurement);
  edge->setInformation(information);
  edge->vertices()[0] = v_plane1;
  edge->vertices()[1] = v_plane2;
  graph->addEdge(edge);
  return edge;
}


// ============================================================================
// ============================================================================
//                          ROBUST KERNELS
// ============================================================================
// ============================================================================
// Robust kernels protect the optimization from OUTLIER measurements.
//
// Without a robust kernel, a single bad loop closure can distort the entire map
// because the optimizer tries hard to satisfy it (quadratic penalty).
//
// A robust kernel replaces the quadratic cost with a function that grows
// more slowly for large errors, effectively down-weighting outliers:
//
//   Standard cost:  ρ(e) = e²           (unbounded growth)
//   Huber cost:     ρ(e) = e²           if |e| < δ      (normal near zero)
//                   ρ(e) = 2δ|e| - δ²   if |e| ≥ δ      (linear for outliers)
//
// Available kernels: Cauchy, DCS, Fair, GemanMcClure, Huber, PseudoHuber,
//                    Saturated, Tukey, Welsch
//
// kernel_size (δ) controls the threshold between "inlier" and "outlier" errors.
// ============================================================================
void GraphSLAM::add_robust_kernel(g2o::HyperGraph::Edge* edge, const std::string& kernel_type, double kernel_size) {
  // "NONE" means no robust kernel — use standard quadratic cost.
  if(kernel_type == "NONE") {
    return;
  }

  // Create the robust kernel from the factory by name.
  g2o::RobustKernel* kernel = robust_kernel_factory->construct(kernel_type);
  if(kernel == nullptr) {
    std::cerr << "warning : invalid robust kernel type: " << kernel_type << std::endl;
    return;
  }

  // Set the kernel's delta parameter (the inlier/outlier threshold).
  kernel->setDelta(kernel_size);

  // Attach the kernel to the edge.
  // The edge must be cast to OptimizableGraph::Edge to access setRobustKernel().
  g2o::OptimizableGraph::Edge* edge_ = dynamic_cast<g2o::OptimizableGraph::Edge*>(edge);
  edge_->setRobustKernel(kernel);
}


// ============================================================================
// ============================================================================
//                          GRAPH OPTIMIZATION
// ============================================================================
// ============================================================================
// This is where the magic happens. The optimizer adjusts ALL node positions
// simultaneously to minimize the total error across ALL edges.
//
// The process:
//   1. Build the Hessian matrix H and gradient vector b from all edges
//   2. Solve H * Δx = -b for the update Δx
//   3. Apply Δx to all nodes
//   4. Repeat until convergence or max iterations reached
//
// The chi2 value (χ²) is the total error — it should decrease after optimization.
// ============================================================================
int GraphSLAM::optimize(int num_iterations) {
  g2o::SparseOptimizer* graph = dynamic_cast<g2o::SparseOptimizer*>(this->graph.get());

  // Don't optimize if the graph is too small (not enough constraints).
  // With < 10 edges, the problem is underdetermined and optimization may diverge.
  if(graph->edges().size() < 10) {
    return -1;
  }

  std::cout << std::endl;
  std::cout << "--- pose graph optimization ---" << std::endl;
  std::cout << "nodes: " << graph->vertices().size() << "   edges: " << graph->edges().size() << std::endl;
  std::cout << "optimizing... " << std::flush;

  // Initialize internal data structures for optimization.
  // This computes the initial error and sets up the solver's matrices.
  std::cout << "init" << std::endl;
  graph->initializeOptimization();
  graph->setVerbose(true);   // Print per-iteration chi2 values

  // Record the chi2 (total error) BEFORE optimization for comparison.
  std::cout << "chi2" << std::endl;
  double chi2 = graph->chi2();

  // Run the optimization for up to num_iterations iterations.
  // It may converge and stop early if the error reduction is below epsilon.
  std::cout << "optimize!!" << std::endl;
  auto t1 = ros::WallTime::now();
  int iterations = graph->optimize(num_iterations);
  auto t2 = ros::WallTime::now();

  // Print optimization summary.
  std::cout << "done" << std::endl;
  std::cout << "iterations: " << iterations << " / " << num_iterations << std::endl;
  std::cout << "chi2: (before)" << chi2 << " -> (after)" << graph->chi2() << std::endl;
  std::cout << "time: " << boost::format("%.3f") % (t2 - t1).toSec() << "[sec]" << std::endl;

  return iterations;  // Returns actual iterations performed
}


// ============================================================================
// ============================================================================
//                        SAVE / LOAD GRAPH
// ============================================================================
// ============================================================================
// The graph can be serialized to disk and loaded back later.
// This enables:
//   - Resuming SLAM from a saved state
//   - Debugging / visualizing the graph offline
//   - Sharing map data between robots
//
// The main .g2o file stores vertices and edges in text format.
// The .kernels file separately stores which edges have robust kernels.
// ============================================================================

// --- Save the current pose graph to a file ---
void GraphSLAM::save(const std::string& filename) {
  g2o::SparseOptimizer* graph = dynamic_cast<g2o::SparseOptimizer*>(this->graph.get());

  std::ofstream ofs(filename);
  graph->save(ofs);     // Saves all vertices and edges in g2o's text format

  // Also save robust kernel assignments to a separate file.
  // Without this, loading would lose information about which edges have kernels.
  g2o::save_robust_kernels(filename + ".kernels", graph);
}

// --- Load a pose graph from a file ---
bool GraphSLAM::load(const std::string& filename) {
  std::cout << "loading pose graph..." << std::endl;
  g2o::SparseOptimizer* graph = dynamic_cast<g2o::SparseOptimizer*>(this->graph.get());

  std::ifstream ifs(filename);
  if(!graph->load(ifs)) {
    return false;         // Failed to parse the graph file
  }

  std::cout << "nodes  : " << graph->vertices().size() << std::endl;
  std::cout << "edges  : " << graph->edges().size() << std::endl;

  // Restore the robust kernels from the companion .kernels file.
  if(!g2o::load_robust_kernels(filename + ".kernels", graph)) {
    return false;
  }

  return true;
}

}  // namespace hdl_graph_slam
