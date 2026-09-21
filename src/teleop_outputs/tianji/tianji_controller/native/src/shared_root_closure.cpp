#include "tianji_qp_ik/shared_root_closure.hpp"
#include "tianji_qp_ik/so3.hpp"
#include <algorithm>
#include <cmath>
namespace tianji_qp_ik {
SharedRootClosedArm closeSharedRootArm(const SharedRootClosureSideGeometry& g,
    const Pose& palm,const Eigen::Vector3d& preferred,
    const std::optional<Eigen::Vector3d>& previous) noexcept {
  SharedRootClosedArm out;constexpr double eps=1e-9;
  const double l1=g.upper_length_m,l2=g.forearm_length_m;
  if(!std::isfinite(l1)||!std::isfinite(l2)||l1<=eps||l2<=eps||
      !g.shoulder_B.allFinite()||!g.tcp_to_wrist_center.position.allFinite()||
      !palm.position.allFinite()||!isProperRotation(palm.rotation)||!preferred.allFinite()||
      (previous&&!previous->allFinite()))return out;
  const Eigen::Vector3d s=g.shoulder_B,w=palm.position+palm.rotation*g.tcp_to_wrist_center.position;
  const double d=(w-s).norm();if(!std::isfinite(d))return out;
  if(d>l1+l2+eps||d<std::abs(l1-l2)-eps){out.status=ClosureStatus::kOutsideWorkspace;return out;}
  Eigen::Vector3d e;
  if(d<=eps) {
    if(std::abs(l1-l2)>eps){out.status=ClosureStatus::kOutsideWorkspace;return out;}
    if(!previous||(*previous-s).norm()<=eps){out.status=ClosureStatus::kBranchUndetermined;return out;}
    e=s+l1*(*previous-s).normalized();
  } else {
    const Eigen::Vector3d n=(w-s)/d;
    const double a=(l1*l1-l2*l2+d*d)/(2*d);
    const Eigen::Vector3d c=s+a*n;
    const double r2=l1*l1-a*a,eps2=2*std::max(l1,l2)*eps+eps*eps;
    if(!std::isfinite(r2)||r2 < -eps2){out.status=ClosureStatus::kOutsideWorkspace;return out;}
    const double r=std::sqrt(std::max(0.,r2));
    if(r<=eps)e=c;
    else {
      Eigen::Vector3d v=preferred-c;v-=n*n.dot(v);
      if(v.norm()<=eps&&previous){v=*previous-c;v-=n*n.dot(v);}
      if(v.norm()<=eps){out.status=ClosureStatus::kBranchUndetermined;return out;}
      e=c+r*v.normalized();
    }
  }
  if(!e.allFinite()||std::abs((e-s).norm()-l1)>eps||std::abs((w-e).norm()-l2)>eps) {
    out.status=ClosureStatus::kResidualFailure;return out;
  }
  out.target.shoulder=s;out.target.elbow=e;out.target.wrist=w;
  out.target.hand=palm.position;out.target.palm=palm;
  out.status=ClosureStatus::kAccepted;return out;
}
bool closeSharedRootTargets(SparkUpperTargets& targets,const SharedRootClosureGeometry& geometry,
                            const SharedRootElbowHistory& history) noexcept {
  auto candidate=targets;
  for(int side=0;side<2;++side) {
    auto& t=side?candidate.right:candidate.left;
    const auto closed=closeSharedRootArm(geometry[side],t.palm,t.elbow,history[side]);
    if(!closed.valid()) {
      targets.valid=false;
      targets.detail=closed.status==ClosureStatus::kOutsideWorkspace?"MappedPalmOutsideGeometricWorkspace":
        closed.status==ClosureStatus::kBranchUndetermined?"ElbowBranchUndetermined":"ClosureInvalid";
      return false;
    }
    // Preserve source/scale diagnostics while replacing final geometry atomically.
    t.shoulder=closed.target.shoulder;t.elbow=closed.target.elbow;
    t.wrist=closed.target.wrist;t.hand=t.palm.position;
  }
  targets=candidate;return true;
}
} // namespace tianji_qp_ik
