#ifndef _PYBIND_DECOMP_TYPE_
#define _PYBIND_DECOMP_TYPE_

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include <decomp_geometry/polyhedron.h>

namespace py = pybind11;

// type caster: Polyhedron <-> Numpy-array
// namespace pybind11 { namespace detail {
//   template<int Dim> struct type_caster<Hyperplane<Dim>>
//   {
//     public:
//       PYBIND11_TYPE_CASTER(Hyperplane<Dim>, _("Hyperplane<Dim>"));

//       bool load(py::handle src, bool convert)
//       {
//         if (!convert && !py::array_t<int>::check_(src))
//           return false;

//         auto buf = py::array_t<int, py::array::c_style | py::array::forcecast>::ensure(src);
//         if (!buf)
//           return false;

//         return true;
//       }

//       //Conversion part 2 (C++ -> Python)
//       // static py::handle cast(const Hyperplane<Dim>& src,
//       //   py::return_value_policy policy, py::handle parent)
//       // {
//       //   py::array a(std::move(src.shape()), std::move(src.strides(true)), src.data() );

//       //   return a.release();
//       // }
//   };

//   template<int Dim> struct type_caster<Polyhedron<Dim>>
//   {
//     public:
//       PYBIND11_TYPE_CASTER(Polyhedron<Dim>, _("Polydedron<Dim>"));

//       bool load(py::handle src, bool convert)
//       {
//         if (!convert && !py::array_t<int>::check_(src))
//           return false;

//         auto buf = py::array_t<int, py::array::c_style | py::array::forcecast>::ensure(src);
//         if (!buf)
//           return false;

//         return true;
//       }

//       // Conversion part 2 (C++ -> Python)
//       static py::handle cast(const Polyhedron<Dim>& src,
//         py::return_value_policy policy, py::handle parent)
//       {
//         py::array a;
        
//         return a.release();
//       }
//   };
// }}


#endif // _PYBIND_DECOMP_TYPE_
