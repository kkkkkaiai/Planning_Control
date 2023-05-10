#include <decomp_util/seed_decomp.h>
#include <decomp_util/decomp_base.h>
#include <decomp_util/line_segment.h>
#include <decomp_util/ellipsoid_decomp.h>
#include <decomp_util/iterative_decomp.h>
#include <decomp_geometry/ellipsoid.h>
#include <decomp_geometry/polyhedron.h>
#include <decomp_geometry/geometric_utils.h>
// #include <pybind_decomp_type.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;


template<int Dim>
void geometric_utils_all(py::module &m){
  m.def("cal_vertices", py::overload_cast<const Polyhedron2D&>(&cal_vertices));
}

namespace decomp_util{

PYBIND11_MODULE(geometric_utils, m) {
  // Optional docstring
  m.doc() = "Geometric Utils Library";
  
  geometric_utils_all<2>(m);
} // end of PYBIND11_MODULE
} // end of decomp_util