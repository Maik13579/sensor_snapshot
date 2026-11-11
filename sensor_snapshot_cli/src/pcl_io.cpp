// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0

// A deliberately small CPython adapter: Python normalizes the ROS byte layout;
// PCL owns the PCD format and serialization. No ROS middleware or NumPy C ABI is
// needed here, keeping this usable with Python/NumPy versions on all three LTSes.
#define PY_SSIZE_T_CLEAN
#include <Python.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>

#include <cstdint>
#include <exception>
#include <limits>

namespace
{
PyObject * save_pcd(PyObject *, PyObject * args)
{
  const char * filename = nullptr;
  unsigned int width = 0, height = 0, point_step = 0;
  PyObject * fields = nullptr;
  Py_buffer data{};
  if (!PyArg_ParseTuple(
      args, "sIIIOy*", &filename, &width, &height, &point_step, &fields, &data))
  {
    return nullptr;
  }
  const auto row_step = static_cast<std::uint64_t>(width) * point_step;
  const auto data_size = row_step * height;
  if (row_step > std::numeric_limits<std::uint32_t>::max() ||
    data_size != static_cast<std::uint64_t>(data.len) || !PyList_Check(fields))
  {
    PyBuffer_Release(&data);
    PyErr_SetString(PyExc_ValueError, "Invalid packed cloud dimensions or fields");
    return nullptr;
  }

  try {
    pcl::PCLPointCloud2 cloud;
    cloud.width = width;
    cloud.height = height;
    cloud.point_step = point_step;
    cloud.row_step = static_cast<std::uint32_t>(row_step);
    cloud.is_bigendian = false;  // The Python adapter supplies little-endian bytes.
    cloud.is_dense = false;
    constexpr std::uint32_t field_sizes[] = {0, 1, 1, 2, 2, 4, 4, 4, 8, 8, 8, 1};
    for (Py_ssize_t i = 0; i < PyList_Size(fields); ++i) {
      const char * name = nullptr;
      unsigned int offset = 0, datatype = 0, count = 0;
      if (!PyArg_ParseTuple(
          PyList_GetItem(fields, i), "sIII", &name, &offset, &datatype, &count))
      {
        PyBuffer_Release(&data);
        return nullptr;
      }
      if (datatype == 0 || datatype > 11 || count == 0 ||
        static_cast<std::uint64_t>(offset) +
        static_cast<std::uint64_t>(count) * field_sizes[datatype] > point_step)
      {
        PyBuffer_Release(&data);
        PyErr_SetString(PyExc_ValueError, "Invalid packed point field");
        return nullptr;
      }
      if (pcl::getFieldSize(static_cast<int>(datatype)) != static_cast<int>(field_sizes[datatype])) {
        PyBuffer_Release(&data);
        PyErr_SetString(PyExc_ValueError, "Installed PCL does not support this point datatype");
        return nullptr;
      }
      pcl::PCLPointField field;
      field.name = name;
      field.offset = offset;
      field.datatype = static_cast<std::uint8_t>(datatype);
      field.count = count;
      cloud.fields.push_back(field);
    }
    const auto * begin = static_cast<const std::uint8_t *>(data.buf);
    cloud.data.assign(begin, begin + data.len);
    const int result = pcl::PCDWriter().writeBinary(filename, cloud);
    PyBuffer_Release(&data);
    if (result < 0) {
      PyErr_SetString(PyExc_RuntimeError, "PCL could not write the PCD file");
      return nullptr;
    }
    Py_RETURN_NONE;
  } catch (const std::exception & error) {
    PyBuffer_Release(&data);
    PyErr_SetString(PyExc_RuntimeError, error.what());
    return nullptr;
  }
}

PyMethodDef methods[] = {
  {"save", save_pcd, METH_VARARGS, "Write a normalized PointCloud2 using PCL's binary PCD writer."},
  {nullptr, nullptr, 0, nullptr}
};

PyModuleDef module = {
  PyModuleDef_HEAD_INIT, "_pcl_io", "Minimal PCL I/O adapter.", -1, methods,
  nullptr, nullptr, nullptr, nullptr
};
}  // namespace

PyMODINIT_FUNC PyInit__pcl_io()
{
  return PyModule_Create(&module);
}
