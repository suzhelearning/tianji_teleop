#include <mujoco/mujoco.h>

#include <array>
#include <cstring>
#include <iostream>
#include <string>

namespace {

constexpr std::array<const char*, 14> kJointNames{
    "Joint1_L", "Joint2_L", "Joint3_L", "Joint4_L", "Joint5_L", "Joint6_L", "Joint7_L",
    "Joint1_R", "Joint2_R", "Joint3_R", "Joint4_R", "Joint5_R", "Joint6_R", "Joint7_R"};

int inspect(const mjModel* model) {
  std::cout << "nq=" << model->nq << " nv=" << model->nv << " njnt=" << model->njnt
            << " nsite=" << model->nsite << '\n';

  bool valid = model->nq == model->nv &&
               model->nq >= static_cast<int>(kJointNames.size());
  for (const char* name : kJointNames) {
    const int id = mj_name2id(model, mjOBJ_JOINT, name);
    if (id < 0) {
      std::cerr << "missing joint: " << name << '\n';
      valid = false;
      continue;
    }
    const auto range_index = static_cast<std::size_t>(2 * id);
    std::cout << name << " id=" << id << " qpos=" << model->jnt_qposadr[id]
              << " dof=" << model->jnt_dofadr[id] << " range=["
              << model->jnt_range[range_index] << ',' << model->jnt_range[range_index + 1]
              << "]\n";
  }

  for (const char* site : {"tcp_L", "tcp_R"}) {
    const int id = mj_name2id(model, mjOBJ_SITE, site);
    std::cout << site << " id=" << id << '\n';
    valid = valid && id >= 0;
  }
  return valid ? 0 : 2;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2 && argc != 4) {
    std::cerr << "usage: " << argv[0] << " MODEL [--save-mjcf OUTPUT]\n";
    return 64;
  }

  char error[1024]{};
  mjModel* model = mj_loadXML(argv[1], nullptr, error, sizeof(error));
  if (model == nullptr) {
    std::cerr << "MuJoCo model load failed: " << error << '\n';
    return 1;
  }

  if (argc == 4) {
    if (std::strcmp(argv[2], "--save-mjcf") != 0) {
      std::cerr << "expected --save-mjcf\n";
      mj_deleteModel(model);
      return 64;
    }
    if (mj_saveLastXML(argv[3], model, error, sizeof(error)) == 0) {
      std::cerr << "MuJoCo MJCF save failed: " << error << '\n';
      mj_deleteModel(model);
      return 1;
    }
  }

  const int result = inspect(model);
  mj_deleteModel(model);
  return result;
}
