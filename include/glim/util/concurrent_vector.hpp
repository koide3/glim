#pragma once

#include <deque>
#include <vector>
#include <mutex>
#include <atomic>
#include <optional>
#include <condition_variable>

namespace glim {

/**
 * @brief Queue data policy
 */
struct DataStorePolicy {
public:
  template <typename T, typename Alloc>
  void regulate(std::deque<T, Alloc>& queue) const {
    if (queue.size() < max_size) {
      return;
    }

    const size_t num_erase = queue.size() - max_size;
    if (pop_front) {
      queue.erase(queue.begin(), queue.begin() + num_erase);
    } else {
      queue.erase(queue.end() - num_erase, queue.end());
    }
  }

  static DataStorePolicy UNLIMITED() { return DataStorePolicy(); }
  static DataStorePolicy UPTO(const size_t max_size, const bool pop_front = true) { return DataStorePolicy{max_size, pop_front}; }

public:
  const size_t max_size = std::numeric_limits<size_t>::max();
  const bool pop_front = true;
};

/**
 * @brief Simple thread-safe vector with mutex-lock.
 * @note  This class is performant in the single-thread-input single-thread-output situation.
 *        In the multi-thread-input multi-thread-output situation, consider using concurrent containers in TBB.
 *
 * @tparam T      Data type
 * @tparam Alloc  Allocator
 */
template <typename T, typename Alloc = std::allocator<T>>
class ConcurrentVector {
public:
  ConcurrentVector(const DataStorePolicy& policy = DataStorePolicy::UNLIMITED()) : policy(policy) { end_of_data = false; }

  void submit_end_of_data() {
    end_of_data = true;
    cond.notify_all();
  }

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
    policy.regulate(values);
    cond.notify_one();
  }

  void clear() {
    std::lock_guard<std::mutex> lock(mutex);
    values.clear();
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
  template <typename Container>
  void insert(const Container& new_values) {
    if (new_values.empty()) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex);
    values.insert(values.end(), new_values.begin(), new_values.end());
    policy.regulate(values);
    cond.notify_all();
  }

  /**
   * @brief Get the first element in the queue
   * @return  nullopt if the queue is empty
   */
  std::optional<T> pop() {
    std::lock_guard<std::mutex> lock(mutex);
    if (values.empty()) {
      return std::nullopt;
    }

    const T data = values.front();
    values.pop_front();
    return data;
  }

  /**
   * @brief Get the first element in the queue.
   *        If the queue is empty, this method waits until a new data arrives or EOD is submitted.
   * @return  nullopt if the queue is empty and EOD is submitted.
   */
  std::optional<T> pop_wait() {
    std::unique_lock<std::mutex> lock(mutex);

    std::optional<T> data;
    cond.wait(lock, [this, &data] {
      if (values.empty()) {
        return static_cast<bool>(end_of_data);
      }

      data = values.front();
      values.pop_front();
      return true;
    });

    return data;
  }

  /**
   * @brief Get all the data and clear the container.
   *        If the queue is empty, this method waits until a new data arrives or EOD is submitted.
   * @return std::vector<T, Alloc>   All data or empty if EOD is submitted.
   */
  std::vector<T, Alloc> get_all_and_clear_wait() {
    std::unique_lock<std::mutex> lock(mutex);

    std::vector<T, Alloc> buffer;
    cond.wait(lock, [this, &buffer] {
      if (values.empty()) {
        return static_cast<bool>(end_of_data);
      }

      buffer.assign(values.begin(), values.end());
      values.clear();
      return true;
    });

    return buffer;
  }

  /**
   * @brief Get all the data and clear the container
   * @return std::vector<T, Alloc>   All data
   */
  std::vector<T, Alloc> get_all_and_clear() {
    std::vector<T, Alloc> buffer;
    std::lock_guard<std::mutex> lock(mutex);
    buffer.assign(values.begin(), values.end());
    values.clear();

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
      buffer.assign(values.begin(), values.end());
      values.clear();
    } else {
      buffer.assign(values.begin(), values.begin() + num_max);
      values.erase(values.begin(), values.begin() + num_max);
    }

    return buffer;
  }

private:
  const DataStorePolicy policy;

  std::atomic_bool end_of_data;
  std::condition_variable cond;

  mutable std::mutex mutex;
  std::deque<T, Alloc> values;
};

}  // namespace  glim
