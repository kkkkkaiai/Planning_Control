#include <decomp_util/seed_decomp.h>

#include <pybind11/stl.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;


template<int Dim>
void seed_decomp(py::module &m){
//     py::class_<DecompBase<T>>(m, "DecompBase");

    py::class_<SeedDecomp<Dim>>(m, "SeedDecomp")
    .def(py::init<>())
    .def(py::init<Vecf<Dim>>(), py::arg("p"))
    .def("dilate", 
         py::overload_cast<double>(&SeedDecomp<Dim>::dilate),
         py::arg("radius")) 
    .def("get_seed", 
         &SeedDecomp<Dim>::get_seed, 
         py::return_value_policy::reference_internal)
    .def("add_local_bbox", 
         py::overload_cast<Polyhedron<Dim>&>(&SeedDecomp<Dim>::add_local_bbox),
         py::arg("Vs"))
    ;
}

namespace decomp_util{

PYBIND11_MODULE(decomp_util_seed, m) {
    // Optional docstring
    m.doc() = "Decomposition Util Library";
    
    seed_decomp<2>(m);
} // end of PYBIND11_MODULE
} // end of decomp_util