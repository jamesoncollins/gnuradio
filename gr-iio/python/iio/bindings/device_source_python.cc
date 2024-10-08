/*
 * Copyright 2020 Free Software Foundation, Inc.
 *
 * This file is part of GNU Radio
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

/***********************************************************************************/
/* This file is automatically generated using bindtool and can be manually edited  */
/* The following lines can be configured to regenerate this file during cmake      */
/* If manual edits are made, the following tags should be modified accordingly.    */
/* BINDTOOL_GEN_AUTOMATIC(0)                                                       */
/* BINDTOOL_USE_PYGCCXML(0)                                                        */
/* BINDTOOL_HEADER_FILE(device_source.h)                                           */
/* BINDTOOL_HEADER_FILE_HASH(6853567a64698c3343e088a2498e7dc2)                     */
/***********************************************************************************/

#include <pybind11/complex.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

#include <gnuradio/iio/device_source.h>
// pydoc.h is automatically generated in the build directory
#include <device_source_pydoc.h>

template <typename T>
void bind_device_source_template(py::module& m, const char* classname)
{

    using device_source = gr::iio::device_source<T>;


    py::class_<device_source,
               gr::sync_block,
               gr::block,
               gr::basic_block,
               std::shared_ptr<device_source>>(m, classname, D(device_source))

        .def(py::init(&device_source::make),
             py::arg("uri"),
             py::arg("device"),
             py::arg("channels"),
             py::arg("device_phy"),
             py::arg("params"),
             py::arg("buffer_size") = DEFAULT_BUFFER_SIZE,
             py::arg("decimation") = 0,
             D(device_source, make))

        .def("set_buffer_size",
             &device_source::set_buffer_size,
             py::arg("buffer_size") = DEFAULT_BUFFER_SIZE,
             D(device_source, set_buffer_size))

        .def("set_timeout_ms",
             &device_source::set_timeout_ms,
             py::arg("timeout"),
             D(device_source, set_timeout_ms))

        .def(
            "set_len_tag_key",
            [](device_source& src, const std::string& str) { src.set_len_tag_key(str); },
            py::arg("len_tag_key") = "");
}

void bind_device_source(py::module& m)
{
     bind_device_source_template<std::int8_t>(m, "device_source_b");
     bind_device_source_template<std::int16_t>(m, "device_source_s");
     bind_device_source_template<std::int32_t>(m, "device_source_i");
     bind_device_source_template<float>(m, "device_source_f");
}
