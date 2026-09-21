#pragma once
#include <algorithm>
#include <cstdint>
#include <stdexcept>

// Offline only. Caller supplies unique, ordered source observations and local
// receive times. No source-clock subtraction and no inferred post-EOF silence.
class SharedRootCoverageTimeline {
 public:
  explicit SharedRootCoverageTimeline(std::int64_t freshness_ns):freshness_(freshness_ns) {
    if(freshness_ns<=0)throw std::invalid_argument("invalid coverage freshness");
  }
  void observe(std::int64_t receive_ns,bool valid) {
    if(receive_ns<0||(initialized_&&receive_ns<cursor_))
      throw std::invalid_argument("nonmonotonic coverage time");
    if(initialized_) advance(receive_ns);
    else {initialized_=true;cursor_=receive_ns;}
    last_=receive_ns;valid_=valid;
    if(valid)run_=0;
  }
  // Explicit evaluation endpoint, normally the final recorded receive time.
  // A recording cannot establish how long the source stayed silent after EOF.
  SharedRootCoverageTimeline through(std::int64_t end_ns) const {
    auto copy=*this;
    if(!initialized_||end_ns<cursor_)throw std::invalid_argument("invalid coverage end");
    copy.advance(end_ns);return copy;
  }
  std::int64_t duration() const {return elapsed_;}
  std::int64_t invalidDuration() const {return invalid_;}
  std::int64_t maximumInvalidDuration() const {return maximum_;}
 private:
  void advance(std::int64_t end_ns) {
    const auto dt=end_ns-cursor_;
    const auto remaining=valid_?std::max<std::int64_t>(0,freshness_-(cursor_-last_)):0;
    const auto invalid_dt=std::max<std::int64_t>(0,dt-remaining);
    elapsed_+=dt;
    invalid_+=invalid_dt;run_+=invalid_dt;maximum_=std::max(maximum_,run_);
    cursor_=end_ns;
  }
  std::int64_t freshness_,last_{0},cursor_{0},elapsed_{0},invalid_{0},run_{0},maximum_{0};
  bool initialized_{false},valid_{false};
};
