// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from checkpoint_interfaces:msg/ParametersToTarget.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "checkpoint_interfaces/msg/detail/parameters_to_target__struct.h"
#include "checkpoint_interfaces/msg/detail/parameters_to_target__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool checkpoint_interfaces__msg__parameters_to_target__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[67];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("checkpoint_interfaces.msg._parameters_to_target.ParametersToTarget", full_classname_dest, 66) == 0);
  }
  checkpoint_interfaces__msg__ParametersToTarget * ros_message = _ros_message;
  {  // marker_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "marker_id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->marker_id = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // alignment_error
    PyObject * field = PyObject_GetAttrString(_pymsg, "alignment_error");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->alignment_error = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // distance_seperation
    PyObject * field = PyObject_GetAttrString(_pymsg, "distance_seperation");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->distance_seperation = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * checkpoint_interfaces__msg__parameters_to_target__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of ParametersToTarget */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("checkpoint_interfaces.msg._parameters_to_target");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "ParametersToTarget");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  checkpoint_interfaces__msg__ParametersToTarget * ros_message = (checkpoint_interfaces__msg__ParametersToTarget *)raw_ros_message;
  {  // marker_id
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->marker_id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "marker_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // alignment_error
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->alignment_error);
    {
      int rc = PyObject_SetAttrString(_pymessage, "alignment_error", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // distance_seperation
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->distance_seperation);
    {
      int rc = PyObject_SetAttrString(_pymessage, "distance_seperation", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
