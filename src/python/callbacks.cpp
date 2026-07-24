#include "pyglim.hpp"

#include <glim/util/callback_slot.hpp>

namespace {

/// @brief Python-friendly wrapper of glim::ScopedCallbackContext usable as a context manager.
///        The callback context is applied on __enter__ and restored on __exit__ so that its
///        lifetime follows the Python `with` block rather than non-deterministic GC timing.
struct PyScopedCallbackContext {
  explicit PyScopedCallbackContext(int id) : id(id), prev_id(glim::CallbackContext::GLOBAL) {}

  PyScopedCallbackContext& enter() {
    prev_id = glim::CallbackContext::current();
    glim::CallbackContext::set(id);
    return *this;
  }

  void exit(const py::object&, const py::object&, const py::object&) { glim::CallbackContext::set(prev_id); }

  int id;
  int prev_id;
};

}  // namespace

void define_callbacks(py::module_& m) {
  // glim::CallbackContext
  py::class_<glim::CallbackContext> callback_context(m, "CallbackContext", "Thread-local context that filters callbacks by ID");
  callback_context.attr("GLOBAL") = glim::CallbackContext::GLOBAL;
  callback_context.def_static("current", &glim::CallbackContext::current, "Get the current thread-local callback context ID");
  callback_context.def_static("set", &glim::CallbackContext::set, py::arg("id"), "Set the current thread-local callback context ID");

  // glim::ScopedCallbackContext (exposed as a context manager)
  py::class_<PyScopedCallbackContext>(m, "ScopedCallbackContext", "Scoped callback context manager (use with a `with` statement)")
    .def(py::init<int>(), py::arg("id"))
    .def("__enter__", &PyScopedCallbackContext::enter)
    .def("__exit__", &PyScopedCallbackContext::exit, py::arg("exc_type"), py::arg("exc_value"), py::arg("traceback"));
}
