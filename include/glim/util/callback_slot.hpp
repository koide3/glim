#pragma once

#include <vector>
#include <algorithm>
#include <functional>

namespace glim {

/**
 * @brief Context for callback execution, allowing to filter callbacks based on ID
 */
class CallbackContext {
public:
  static constexpr int GLOBAL = -1;
  static int current() { return current_id; }
  static void set(int id) { current_id = id; }

private:
  static thread_local int current_id;
};

/**
 * @brief Scoped context for callback execution
 */
class ScopedCallbackContext {
public:
  ScopedCallbackContext(int id) : prev_id(CallbackContext::current()) { CallbackContext::set(id); }
  ~ScopedCallbackContext() { CallbackContext::set(prev_id); }

  ScopedCallbackContext(const ScopedCallbackContext&) = delete;
  ScopedCallbackContext& operator=(const ScopedCallbackContext&) = delete;

private:
  const int prev_id;
};

/**
 * @brief Callback slot to hold and trigger multiple callbacks
 * @note  add/remove are not thread-safe, but call is thread-safe
 */
template <typename Func>
class CallbackSlot {
public:
  CallbackSlot() {}
  ~CallbackSlot() {}

  /**
   * @brief Add a new callback
   * @param callback    Callback to be registered
   * @return int        Callback ID
   */
  int add(const std::function<Func>& callback) { return add(callback, CallbackContext::current()); }

  /**
   * @brief Add a new callback with a specific context
   * @param callback    Callback to be registered
   * @param context     Context ID for the callback
   * @return int        Callback ID
   */
  int add(const std::function<Func>& callback, int context) {
    callbacks.emplace_back(context, callback);
    return callbacks.size() - 1;
  }

  /**
   * @brief Remove a callback
   * @param callback_id  Callback ID
   */
  void remove(int callback_id) { callbacks[callback_id] = {CallbackContext::GLOBAL, nullptr}; }

  /**
   * @brief Check if the slot has a valid callback
   * @return true   The slot has at least one valid callback
   * @return false  No valid callbacks
   */
  operator bool() const {
    return !callbacks.empty() && std::any_of(callbacks.begin(), callbacks.end(), [](const auto& cb) { return cb.second; });
  }

  /**
   * @brief Call all the registered callbacks
   * @param args  Arguments for the callbacks
   */
  template <class... Args>
  void call(Args&&... args) const {
    if (callbacks.empty()) {
      return;
    }

    const int ctx = CallbackContext::current();
    for (const auto& callback : callbacks) {
      const int cb_ctx = callback.first;
      const auto& cb_func = callback.second;
      if (!cb_func) {
        continue;
      }

      if (ctx == cb_ctx || ctx == CallbackContext::GLOBAL || cb_ctx == CallbackContext::GLOBAL) {
        cb_func(args...);
      }
    }
  }

  /**
   * @brief Call all the registered callbacks
   * @param args  Arguments for the callbacks
   */
  template <class... Args>
  void operator()(Args&&... args) const {
    return call(args...);
  }

private:
  std::vector<std::pair<int, std::function<Func>>> callbacks;  // (ctx_id, callback)
};

}  // namespace glim
