#pragma once

#include "tianji_qp_ik/config.hpp"
#include "tianji_qp_ik/types.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace tianji_qp_ik {

enum class BenchmarkCategory : std::size_t {
  kNominal = 0,
  kJointLimit,
  kSingular,
  kUnreachable,
  kContinuous,
};

inline constexpr std::size_t kBenchmarkCategoryCount = 5U;

struct BenchmarkSample {
  QpProblem7 left;
  QpProblem7 right;
  BenchmarkCategory category{BenchmarkCategory::kNominal};
};

Vec7 safeSingularBenchmarkPosition(ArmSide side);
std::vector<BenchmarkSample> generateBenchmarkDataset(const std::string& model_path,
                                                       const QpIkConfig& config,
                                                       std::size_t sample_count,
                                                       std::uint32_t seed);
std::uint64_t hashBenchmarkDataset(const std::vector<BenchmarkSample>& samples);
std::array<std::size_t, kBenchmarkCategoryCount> benchmarkCategoryCounts(
    const std::vector<BenchmarkSample>& samples);
std::string benchmarkCategoryName(BenchmarkCategory category);

}  // namespace tianji_qp_ik
