#include <array>
#include <cmath>
#include <cstddef>
#include <new>

namespace {
struct Filter {
  double alpha;
  std::array<double,20> value{};
  bool initialized=false;
  bool single_precision=false;
};
}
// Per-instance single-owner ABI, usable from both pinned Python environments.
// No global filter state, hardware dependencies or exceptions across the C ABI.
extern "C" {
int tianji_hand_filter_abi() noexcept {return 1;}
void* tianji_hand_filter_create(double alpha) noexcept {
  if(!std::isfinite(alpha) || alpha<=0 || alpha>1) return nullptr;
  return new(std::nothrow) Filter{alpha,{},false,false};
}
void tianji_hand_filter_destroy(void* handle) noexcept {delete static_cast<Filter*>(handle);}
int tianji_hand_filter_reset(void* handle) noexcept {
  if(!handle) return -1;
  auto& filter=*static_cast<Filter*>(handle);filter.initialized=false;filter.value.fill(0);return 0;
}
int tianji_hand_filter_next(void* handle,const double* input,std::size_t count,double* output,int single_precision) noexcept {
  if(!handle || !input || !output || count!=20 || (single_precision!=0 && single_precision!=1)) return -1;
  auto& filter=*static_cast<Filter*>(handle);
  const bool use_float=single_precision && (!filter.initialized || filter.single_precision);
  std::array<double,20> next{};
  for(std::size_t i=0;i<20;++i) {
    if(!std::isfinite(input[i])) return -1;
    if(filter.initialized && use_float) {
      const float delta=static_cast<float>(input[i])-static_cast<float>(filter.value[i]);
      const float weighted=static_cast<float>(filter.alpha)*delta;
      next[i]=static_cast<float>(filter.value[i])+weighted;
    } else next[i]=filter.initialized?filter.value[i]+filter.alpha*(input[i]-filter.value[i]):input[i];
    if(!std::isfinite(next[i])) return -1;
  }
  filter.value=next;filter.initialized=true;filter.single_precision=use_float;
  for(std::size_t i=0;i<20;++i) output[i]=next[i];
  return 0;
}
}
