#include <pybind11/pybind11.h>

void bind_morphology(pybind11::module_& module);
void bind_encoder_kinematics(pybind11::module_& module);

PYBIND11_MODULE(_native, module) {
    module.doc() = "Native exoskeleton morphology and encoder kinematics kernels";
    bind_morphology(module);
    bind_encoder_kinematics(module);
}
