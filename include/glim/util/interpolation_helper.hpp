#pragma once

#include <deque>
#include <algorithm>

namespace glim {

enum class InterpolationHelperResult { SUCCESS, FAILURE, WAITING };
enum class InterpolationHelperSearchMode { LINEAR, BINARY };

template <typename Value>
class InterpolationHelper {
public:
  using StampedValue = std::pair<double, Value>;

  InterpolationHelper(InterpolationHelperSearchMode search_mode = InterpolationHelperSearchMode::LINEAR) : search_mode(search_mode) {}
  ~InterpolationHelper() {}

  int size() const { return values.size(); }

  void add(double stamp, const Value& value) {
    if (!values.empty() && values.back().first > stamp) {
      std::cerr << "inserting non-ordered values!!" << std::endl;
      return;
    }
    values.emplace_back(stamp, value);
  }

  void add(const std::pair<double, Value>& stamped_value) {  //
    values.emplace_back(stamped_value.first, stamped_value.second);
  }

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
