#pragma once

#include <vector>

#ifndef Q_MOC_RUN
#include <boost/python.hpp>
#include <boost/python/refcount.hpp>
#endif

template <typename T>
struct Vector_T_to_python
{
   static PyObject* convert(const std::vector<T> &v)
   {
      return
         boost::python::incref(
class_<std::vector<X> >("XVec")
        .def(vector_indexing_suite<std::vector<X> >())
    ;

