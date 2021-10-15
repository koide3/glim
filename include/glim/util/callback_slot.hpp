#pragma once

#include <vector>
#include <functional>

template <typename Func>
class CallbackSlot {
public:
  CallbackSlot() {}
  ~CallbackSlot() {}

  int add(const std::function<Func>& callback) {
    callbacks.push_back(callback);
    return callbacks.size() - 1;
  }

  void erase(int callback_id) { callbacks[callback_id] = nullptr; }

  template <class... Args>
  void call(const Args... args) const {
    for (const auto& callback : callbacks) {
      if (callback) {
        callback(args...);
      }
    }
  }

  template <class... Args>
  void operator()(const Args... args) const {
    return call(args...);
  }

private:
  std::vector<std::function<Func>> callbacks;
};
