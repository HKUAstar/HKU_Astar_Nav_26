// Phase 3: rog_planner — SentryMap
//
// Trivial subclass of rog_map::ROGMap that exposes ESDF queries (distance,
// gradient) and the writer-side mutex to planner code. ROG-Map keeps the
// underlying ESDFMap pointer in `protected esdf_map_`; the parent class
// already exposes `getDistance(Vec3f)` publicly via prob_map.h, but the
// gradient call lives on ESDFMap itself, so we forward it here.
//
// Thread safety: the 1 kHz updateESDF3D() writer holds the same mutex
// returned by `getEsdfMutex()`. Planner code MUST hold this mutex around
// any sequence of getDistance/evaluateFirstGrad calls that needs a
// consistent snapshot (e.g., the entire L-BFGS optimize() outer loop).
//
// MIT-licensed (matches rog_planner package). DO NOT add GPL code here.

#pragma once

#include <mutex>
#include <Eigen/Core>
#include <ros/ros.h>
#include <rog_map/rog_map.h>

namespace rog_planner {

class SentryMap : public rog_map::ROGMap {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Vec3 = Eigen::Vector3d;

  explicit SentryMap(const ros::NodeHandle& nh) : rog_map::ROGMap(nh) {}

  // Distance is already public on ProbMap (the parent of ROGMap): we only
  // re-expose it here for symmetry. Caller must hold getEsdfMutex() if a
  // racy read against the 1 kHz writer is unacceptable.
  inline double dist(const Vec3& p) const {
    return rog_map::ROGMap::getDistance(p);
  }

  // ESDF gradient via the protected esdf_map_ pointer. Same locking
  // requirement as dist().
  inline void grad(const Vec3& p, Vec3& g) {
    esdf_map_->evaluateFirstGrad(p, g);
  }

  // Combined distance + gradient with a single trilinear lookup batch.
  // Currently delegates to the two separate calls; refactor later if hot.
  inline void distAndGrad(const Vec3& p, double& d, Vec3& g) {
    d = rog_map::ROGMap::getDistance(p);
    esdf_map_->evaluateFirstGrad(p, g);
  }

  // Writer-side mutex. Held internally by ESDFMap::updateESDF3D() during
  // each 1 kHz update. Planner holds this around the L-BFGS optimize()
  // call to eliminate torn reads.
  inline std::mutex& getEsdfMutex() {
    return esdf_map_->getUpdateMtx();
  }
};

}  // namespace rog_planner
