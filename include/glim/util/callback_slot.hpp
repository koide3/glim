#pragma once

#include <vector>
#include <algorithm>
#include <functional>

/**
 * @brief Callback slot to hold and trigger multiple callbacks
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
  int add(const std::function<Func>& callback) {
    callbacks.push_back(callback);
    return callbacks.size() - 1;
  }

  /**
   * @brief Remove a callback
   * @param callback_id  Callback ID
   */
  void remove(int callback_id) { callbacks[callback_id] = nullptr; }

  /**
   * @brief Check if the slot has a valid callback
   * @return true   The slot has at least one valid callback
   * @return false  No valid callbacks
   */
  operator bool() const {
    return !callbacks.empty() && std::any_of(callbacks.begin(), callbacks.end(), [](const std::function<Func>& f) { return f; });
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

    for (const auto& callback : callbacks) {
      if (callback) {
        callback(args...);
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
  std::vector<std::function<Func>> callbacks;
};
