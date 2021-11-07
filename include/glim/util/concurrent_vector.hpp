#pragma once

#include <mutex>
#include <vector>

namespace glim {

/**
 * @brief Simple thread-safe vector with mutex-lock
 *
 * @tparam T      Data type
 * @tparam Alloc  Allocator
 */
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

  void reserve(int n) {
    std::lock_guard<std::mutex> lock(mutex);
    values.reserve(n);
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

  /**
   * @brief Insert new_values at the end of the container
   * @param new_values  Values to be inserted
   */
  void insert(const std::vector<T, Alloc>& new_values) {
    if (new_values.empty()) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex);
    values.insert(values.end(), new_values.begin(), new_values.end());
  }

  /**
   * @brief Get all the data and clear the container
   * @return std::vector<T, Alloc>   All data
   */
  std::vector<T, Alloc> get_all_and_clear() {
    std::vector<T, Alloc> buffer;
    std::lock_guard<std::mutex> lock(mutex);
    buffer.swap(values);
    return buffer;
  }

  /**
   * @brief Get up to N data and erase them from the container
   * @param num_max   Maximum number of data
   * @return std::vector<T, Alloc>  Up to num_max data
   */
  std::vector<T, Alloc> get_and_clear(int num_max) {
    std::vector<T, Alloc> buffer;
    std::lock_guard<std::mutex> lock(mutex);
    if (values.size() <= num_max) {
      buffer.swap(values);
    } else {
      buffer.assign(values.begin(), values.begin() + num_max);
      values.erase(values.begin(), values.begin() + num_max);
    }

    return buffer;
  }

private:
  mutable std::mutex mutex;
  std::vector<T, Alloc> values;
};

}  // namespace  glim
