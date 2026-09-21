#pragma once
#include "raw_input.hpp"
#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>
namespace tianji_control {
// Mapped-palm Z-only or opt-in X/Z sampler. No target writes or command authority.
// Successful sampling is only a candidate. The owner commits after worker ACK.
class HeightCalibration {
 public:
  struct Status {
    std::string state="uncalibrated",error;
    std::array<std::uint64_t,2> count{};
    std::optional<std::array<double,2>> offsets,means;
    std::optional<std::array<double,2>> x_offsets,x_means;
  };
  explicit HeightCalibration(std::array<double,2> reference,
      std::optional<std::array<double,2>> x_reference=std::nullopt,bool common_x=false):reference_(reference),x_reference_(x_reference) {
    for(double z:reference) if(!std::isfinite(z)) throw std::invalid_argument("finite bilateral TCP heights required");
    if(x_reference_) for(double x:*x_reference_) if(!std::isfinite(x)) throw std::invalid_argument("finite TCP X required");
    if(common_x) {
      if(!x_reference_) throw std::invalid_argument("common X requires X/Z calibration");
      const double x=std::min((*x_reference_)[0],(*x_reference_)[1]);
      x_reference_=std::array<double,2>{x,x};
    }
  }
  void begin(std::int64_t now) {
    if(now<0) throw std::invalid_argument("nonnegative calibration clock required");
    started_=now; status_.state="collecting"; status_.error.clear(); status_.count={};
    stats_={}; identity_=false; sequence_=0; candidate_.reset(); candidate_means_.reset();
    candidate_x_.reset();
  }
  void fail(const std::string& reason) { status_.state="failed"; status_.error=reason; candidate_.reset(); candidate_x_.reset(); }
  const Status& status() const { return status_; }
  bool ready() const { return status_.offsets.has_value() && status_.state!="collecting" && status_.state!="sampled"; }
  const std::optional<std::array<double,2>>& candidate() const { return candidate_; }
  const std::optional<std::array<double,2>>& candidate_x() const { return candidate_x_; }
  void add(const RawProgress& p,std::int64_t stamp,std::int64_t now) {
    if(status_.state!="collecting" || !p.accepted) return;
    if(identity_ && (p.epoch!=epoch_ || p.generation!=generation_)) { fail("tracking identity/epoch changed during calibration"); return; }
    identity_=true; epoch_=p.epoch; generation_=p.generation;
    if(p.sequence<=sequence_) return;
    sequence_=p.sequence;
    for(int s=0;s<2 && status_.state=="collecting";++s) {
      if(!p.skeleton_valid || !p.rotations_valid) { fail("wrist tracking lost during calibration"); break; }
      if(stamp<0 || now<stamp || now-stamp>gap) { fail("stale calibration input"); break; }
      if(stamp<started_) continue;
      auto& v=stats_[s];
      if(stamp-(v.count?v.last:started_)>gap) { fail("calibration input gap exceeds 0.25 s"); break; }
      if(v.count && stamp<=v.last) continue;
      if(!v.count) { v.first=stamp; v.minimum=v.maximum=p.palms[s]; }
      v.last=stamp; ++v.count; status_.count[s]=v.count;
      for(int a=0;a<3;++a) {
        v.finite=v.finite && std::isfinite(p.palms[s][a]);
        v.minimum[a]=std::min(v.minimum[a],p.palms[s][a]);
        v.maximum[a]=std::max(v.maximum[a],p.palms[s][a]);
      }
      v.sum_z+=p.palms[s][2];
      v.sum_x+=p.palms[s][0];
    }
  }
  bool tick(std::int64_t now) {
    if(status_.state!="collecting") return false;
    if(now<started_) { fail("calibration clock rollback"); return false; }
    for(const auto& v:stats_) if(now-(v.count?v.last:started_)>gap) { fail("calibration input gap exceeds 0.25 s"); return false; }
    if(now-started_<2000000000LL) return false;
    std::array<double,2> means{},offsets{};
    for(int s=0;s<2;++s) {
      const auto& v=stats_[s];
      if(v.count<30 || v.last-v.first<1500000000LL) { fail("insufficient distinct calibration frames"); return false; }
      bool moved=!v.finite;
      for(int a=0;a<3;++a) moved=moved || v.maximum[a]-v.minimum[a]>.06;
      if(moved) { fail("wrist moved too much; hold horizontal pose steadily"); return false; }
      means[s]=v.sum_z/v.count; offsets[s]=reference_[s]-means[s];
    }
    for(double z:offsets) if(!std::isfinite(z) || std::abs(z)>1.) { fail("height offset exceeds 1 m; check pose and calibration"); return false; }
    if(x_reference_) {
      std::array<double,2> x{};
      for(int s=0;s<2;++s) {
        x[s]=(*x_reference_)[s]-stats_[s].sum_x/stats_[s].count;
        if(!std::isfinite(x[s]) || std::abs(x[s])>1.) { fail("X offset exceeds 1 m"); return false; }
      }
      candidate_x_=x;
    }
    candidate_=offsets; candidate_means_=means; status_.state="sampled"; return true;
  }
  void commit() {
    if(status_.state!="sampled" || !candidate_) throw std::logic_error("no height candidate to commit");
    status_.offsets=candidate_; status_.means=candidate_means_; status_.state="calibrated"; status_.error.clear();
    status_.x_offsets=candidate_x_;
    if(candidate_x_) status_.x_means=std::array<double,2>{stats_[0].sum_x/stats_[0].count,stats_[1].sum_x/stats_[1].count};
    candidate_.reset(); candidate_x_.reset();
  }
 private:
  struct Stats { std::uint64_t count=0; std::int64_t first=0,last=0; double sum_z=0,sum_x=0;
                 std::array<double,3> minimum{},maximum{}; bool finite=true; };
  static constexpr std::int64_t gap=250000000;
  std::array<double,2> reference_;
  std::optional<std::array<double,2>> x_reference_,candidate_x_;
  std::array<Stats,2> stats_{};
  Status status_;
  std::optional<std::array<double,2>> candidate_,candidate_means_;
  std::int64_t started_=0;
  std::uint64_t epoch_=0,generation_=0,sequence_=0;
  bool identity_=false;
};
}
