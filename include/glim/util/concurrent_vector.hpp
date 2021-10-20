#pragma once

#include <mutex>
#include <vector>

namespace glim {

template <typename T, typename Alloc = std::allocator<T>>
class ConcurrentVector {
public:

  bool empty() const {
    std::lock_guard<std::mutex> lock(mutex);
    return values.empty();
  }

  int size() const {
    std::lock_guard<std::mutex> lock(mutex);
    return values.size();
  }

  void push_back(const T& value) {
    std::lock_guard<std::mutex> lock(mutex);
    values.push_back(value);
  }

  T front() const {
    std::lock_guard<std::mutex> lock(mutex);
    return values.front();
  }

  T back() const {
    std::lock_guard<std::mutex> lock(mutex);
    return values.back();
  }

  void insert(const std::vector<T, Alloc>& new_values) {
    if(new_values.empty()) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex);
    values.insert(values.end(), new_values.begin(), new_values.end());
  }

  std::vector<T, Alloc> get_all_and_clear() {
    std::vector<T, Alloc> buffer;
    std::lock_guard<std::mutex> lock(mutex);
    buffer.swap(values);
    return buffer;
  }

private:
  mutable std::mutex mutex;
  std::vector<T, Alloc> values;
};

}  // namespace  glim
