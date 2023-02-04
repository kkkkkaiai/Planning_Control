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
class PyDecompBase: public DecompBase<Dim>{
public:
  using DecompBase<Dim>::DecompBase;
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
void decomp_util_3D(py::module &m){
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

  py::class_<Ellipsoid<Dim>>(m, "Ellipsoid")
  .def(py::init<>())
  .def(py::init<const Matf<Dim, Dim>&, const Vecf<Dim>&>())
  .def("dist",
       &Ellipsoid<Dim>::dist)
  .def("points_inside",
       &Ellipsoid<Dim>::points_inside)
  .def("closest_point",
       &Ellipsoid<Dim>::closest_point)
  .def("closest_hyperplane",
       &Ellipsoid<Dim>::closest_hyperplane)
  .def("print",
       &Ellipsoid<Dim>::print)
  .def("volume",
       &Ellipsoid<Dim>::volume)
  .def("C",
       &Ellipsoid<Dim>::C)
  .def("d",
       &Ellipsoid<Dim>::d)
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

  py::class_<SeedDecomp<Dim>, DecompBase<Dim>>(m, "SeedDecomp3D")
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
  .def("get_line_segment",
        &LineSegment<Dim>::get_line_segment)
  .def("add_local_bbox",
        py::overload_cast<Polyhedron<Dim>&>(&LineSegment<Dim>::add_local_bbox),
        py::arg("Vs"))
  ;
  
  py::class_<LineSegment3D<Dim>, LineSegment<Dim>>(m, "LineSegment3D")
  .def(py::init<>())
  .def(py::init<const Vecf<Dim>&, const Vecf<Dim>&>())
  .def("dilate", 
       &LineSegment3D<Dim>::dilate) 
  .def("find_ellipsoid",
       &LineSegment3D<Dim>::find_ellipsoid)
  ;
  
  py::class_<EllipsoidDecomp<Dim>>(m, "EllipsoidDecomp")
  .def(py::init<>())
  .def(py::init<const Vecf<Dim>&, const Vecf<Dim>&>())
  .def("set_obs",
       &EllipsoidDecomp<Dim>::set_obs)
  .def("set_local_bbox",
       &EllipsoidDecomp<Dim>::set_local_bbox)
  .def("get_path",
       &EllipsoidDecomp<Dim>::get_path)
  .def("get_polyhedrons",
       &EllipsoidDecomp<Dim>::get_polyhedrons)
  .def("get_ellipsoids",
       &EllipsoidDecomp<Dim>::get_ellipsoids)
  .def("get_constraints",
       &EllipsoidDecomp<Dim>::get_constraints)
  .def_readwrite("path_", &EllipsoidDecomp<Dim>::path_)
  .def_readwrite("obs_", &EllipsoidDecomp<Dim>::obs_)
  .def_readwrite("ellipsoids_", &EllipsoidDecomp<Dim>::ellipsoids_)
  .def_readwrite("polyhedrons_", &EllipsoidDecomp<Dim>::polyhedrons_)
  .def_readwrite("local_bbox_", &EllipsoidDecomp<Dim>::local_bbox_)
  .def_readwrite("global_bbox_min_", &EllipsoidDecomp<Dim>::global_bbox_min_)
  .def_readwrite("global_bbox_max_", &EllipsoidDecomp<Dim>::global_bbox_max_)
  ;

  py::class_<EllipsoidDecomp3D<Dim>, EllipsoidDecomp<Dim>>(m, "EllipsoidDecomp3D")
  .def(py::init<>())
  .def(py::init<const Vecf<Dim> &, const Vecf<Dim> &>())
  .def("dilate", 
       &EllipsoidDecomp3D<Dim>::dilate) 
  .def("add_global_bbox",
       &EllipsoidDecomp3D<Dim>::add_global_bbox)
  ;

  py::class_<IterativeDecomp3D<Dim>, EllipsoidDecomp3D<Dim>>(m, "IterativeDecomp3D")
  .def(py::init())
  .def(py::init<const Vecf<Dim>&, const Vecf<Dim>&>())
  .def("dilate_iter",
       &IterativeDecomp3D<Dim>::dilate_iter)
  .def("downsample",
       &IterativeDecomp3D<Dim>::downsample)
  .def("cal_closest_dist",
       &IterativeDecomp3D<Dim>::cal_closest_dist)
  .def("simplify",
       &IterativeDecomp3D<Dim>::simplify)
  ;
}

namespace decomp_util{

PYBIND11_MODULE(decomp_util_3d, m) {
  // Optional docstring
  m.doc() = "Decomposition 3D Utils Library";
  
  decomp_util_3D<3>(m);
} // end of PYBIND11_MODULE
} // end of decomp_util