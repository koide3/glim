#pragma once

#include <deque>
#include <algorithm>

namespace glim {

/// @brief Interpolation helper result
enum class InterpolationHelperResult {
  SUCCESS,  ///< Successfully found values that cover the target timestamp
  FAILURE,  ///< Failed because all values are newer than the target timestamp
  WAITING   ///< Waiting because all values are older than the target timestamp
};

/// @brief Interpolation data search mode
enum class InterpolationHelperSearchMode {
  LINEAR,  ///< Linear search
  BINARY   ///< Binary search
};

/// @brief A helper class to find the values that cover a given timestamp from a data stream.
template <typename Value>
class InterpolationHelper {
public:
  using StampedValue = std::pair<double, Value>;

  /// @brief Constructor.
  /// @param search_mode  Search mode
  InterpolationHelper(InterpolationHelperSearchMode search_mode = InterpolationHelperSearchMode::LINEAR) : search_mode(search_mode) {}
  ~InterpolationHelper() {}

  /// @brief Check if it's emptry.
  bool empty() const { return values.empty(); }

  /// @brief Number of queued values.
  int size() const { return values.size(); }

  /// @brief Oldest timestamp in the queue
  double leftmost_time() const { return values.empty() ? 0.0 : values.front().first; }

  /// @brief Newest timestamp in the queue
  double rightmost_time() const { return values.empty() ? 0.0 : values.back().first; }

  /// @brief Add a new value.
  /// @param stamp  Timestamp
  /// @param value  Value
  void add(double stamp, const Value& value) {
    if (!values.empty() && values.back().first > stamp) {
      std::cerr << "inserting non-ordered values!!" << std::endl;
      return;
    }
    values.emplace_back(stamp, value);
  }

  /// @brief Add a new value.
  void add(const std::pair<double, Value>& stamped_value) {  //
    values.emplace_back(stamped_value.first, stamped_value.second);
  }

  /// @brief Find the values that cover the target timestamp.
  /// @param stamp                Target timestamp
  /// @param left_ptr             [out] The closest value that is older than the target timestamp (nullptr if not needed)
  /// @param right_ptr            [out] The closest value that is newer than the target timestamp (nullptr if not needed)
  /// @param remove_cursor_ptr    [out] The index of the value that is older than the left_ptr    (nullptr if not needed)
  /// @return                     Seach result status
  InterpolationHelperResult find(const double stamp, StampedValue* left_ptr, StampedValue* right_ptr, int* remove_cursor_ptr) const {
    if (values.empty() || values.back().first < stamp) {
      return InterpolationHelperResult::WAITING;
    }

    if (values.front().first > stamp) {
      return InterpolationHelperResult::FAILURE;
    }

    int right = 1;
    switch (search_mode) {
      case InterpolationHelperSearchMode::LINEAR:
        while (values[right].first < stamp) {
          right++;
        }
        break;
      case InterpolationHelperSearchMode::BINARY:
        const auto found = std::lower_bound(values.begin(), values.end(), stamp, [](const auto& value, double stamp) { return value.first < stamp; });
        right = std::distance(values.begin(), found);
        break;
    }

    int left = right - 1;

    if (values[left].first > stamp || values[right].first < stamp || right >= values.size()) {
      std::cerr << "error: invalid condition!!" << std::endl;
      abort();
    }

    if (left_ptr) {
      *left_ptr = values[left];
    }
    if (right_ptr) {
      *right_ptr = values[right];
    }
    if (remove_cursor_ptr) {
      *remove_cursor_ptr = left - 1;
    }

    return InterpolationHelperResult::SUCCESS;
  }

  /// @brief Erase values older than the given cursor.
  /// @param remove_cursor  Cursor index
  void erase(int remove_cursor) {
    if (remove_cursor <= 0) {
      return;
    }
    values.erase(values.begin(), values.begin() + remove_cursor);
  }

private:
  InterpolationHelperSearchMode search_mode;
  std::deque<StampedValue> values;
};

}  // namespace glim
