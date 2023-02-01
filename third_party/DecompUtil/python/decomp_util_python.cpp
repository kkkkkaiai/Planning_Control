#include <decomp_util/seed_decomp.h>
#include <decomp_util/decomp_base.h>
#include <decomp_util/line_segment.h>
#include <decomp_geometry/ellipsoid.h>
#include <decomp_geometry/polyhedron.h>
// #include <pybind_decomp_type.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

template<int Dim>
class PyDecompBase: public DecompBase<Dim>{
public:
  using DecompBase<Dim>::DecompBase;
  void dilate(decimal_t radius=0) override{
    PYBIND11_OVERRIDE_PURE(
          void,
          DecompBase<Dim>,
          dialte,
          radius
    );
  }
  void shrink(double shrink_distance) override {
    PYBIND11_OVERRIDE_PURE(
          void,
          DecompBase<Dim>,
          shrink,
          shrink_distance
    );
  }
  void add_local_bbox(Polyhedron<Dim> &Vs) override {
    PYBIND11_OVERRIDE_PURE(
          void,
          DecompBase<Dim>,
          add_local_bbox,
          Vs
    );
  }
};

template<int Dim>
void decomp_util_all(py::module &m){
  py::class_<Hyperplane<Dim>>(m, "Hyperplane")
  .def(py::init<>())
  .def(py::init<const Vecf<Dim>&, const Vecf<Dim>&>())
  .def("signed_dist",
       &Hyperplane<Dim>::signed_dist)
  .def("dist",
       &Hyperplane<Dim>::dist)
  .def_readwrite("p_", &Hyperplane<Dim>::p_)
  .def_readwrite("n_", &Hyperplane<Dim>::n_)
  ;

  py::class_<Polyhedron<Dim>>(m, "Polyhedron")
  .def(py::init<>())
  .def(py::init<const vec_E<Hyperplane<Dim>>&>())
  .def("add",
       &Polyhedron<Dim>::add)
  .def("inside",
       &Polyhedron<Dim>::inside)
  .def("points_inside",
       &Polyhedron<Dim>::points_inside)
  .def("cal_normals",
       &Polyhedron<Dim>::cal_normals)
  .def("hyperplanes",
       &Polyhedron<Dim>::hyperplanes)
  .def_readwrite("vs_", &Polyhedron<Dim>::vs_)
  ;

  py::class_<DecompBase<Dim>, PyDecompBase<Dim>>(m, "DecompBase")
  .def(py::init<>())
  .def("set_local_bbox",
        py::overload_cast<const Vecf<Dim>&>(&DecompBase<Dim>::set_local_bbox),
        py::arg("bbox"))
  .def("set_obs",
        py::overload_cast<const vec_Vecf<Dim>&>(&DecompBase<Dim>::set_obs),
        py::arg("obs"))
  .def("get_obs",
        &DecompBase<Dim>::get_obs,
        py::return_value_policy::reference_internal)
  .def("get_ellipsoid",
        &DecompBase<Dim>::get_ellipsoid,
        py::return_value_policy::reference)
  .def("get_polyhedron",
        &DecompBase<Dim>::get_polyhedron,
        py::return_value_policy::reference)
  .def("find_polyhedron",
        &DecompBase<Dim>::find_polyhedron)
  ;

  py::class_<SeedDecomp<Dim>, DecompBase<Dim>>(m, "SeedDecomp")
  .def(py::init<>())
  .def(py::init<const Vecf<Dim>&>(), py::arg("p"))
  .def("dilate", 
        py::overload_cast<double>(&SeedDecomp<Dim>::dilate),
        py::arg("radius")) 
  .def("get_seed", 
        &SeedDecomp<Dim>::get_seed, 
        py::return_value_policy::reference_internal)
  .def("set_p",
        &SeedDecomp<Dim>::set_p)
  .def("add_local_bbox", 
        py::overload_cast<Polyhedron<Dim>&>(&SeedDecomp<Dim>::add_local_bbox),
        py::arg("Vs"))
  ;

  py::class_<LineSegment<Dim>, DecompBase<Dim>>(m, "LineSegment")
  .def(py::init<>())
  .def(py::init<const Vecf<Dim>&, const Vecf<Dim>&>())
  .def("dilate", 
        py::overload_cast<double>(&LineSegment<Dim>::dilate),
        py::arg("radius")) 
  .def("get_line_segment",
        &LineSegment<Dim>::get_line_segment)
  .def("add_local_bbox",
        py::overload_cast<Polyhedron<Dim>&>(&LineSegment<Dim>::add_local_bbox),
        py::arg("Vs"))
  .def("find_ellipsoid",
       &LineSegment<Dim>::find_ellipsoid)
  ;
}

namespace decomp_util{

PYBIND11_MODULE(decomp_util, m) {
  // Optional docstring
  m.doc() = "Decomposition Util Library";
  
  decomp_util_all<2>(m);
} // end of PYBIND11_MODULE
} // end of decomp_util