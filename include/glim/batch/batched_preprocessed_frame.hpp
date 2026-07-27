#pragma once

#include <memory>
#include <glim/preprocess/preprocessed_frame.hpp>
#include <glim/batch/batched_raw_points.hpp>

namespace glim {

class BatchedPreprocessedFrame {
public:
  using Ptr = std::shared_ptr<BatchedPreprocessedFrame>;
  using ConstPtr = std::shared_ptr<const BatchedPreprocessedFrame>;

  BatchedPreprocessedFrame(int batch_size);
  ~BatchedPreprocessedFrame();

  int size() const;

  PreprocessedFrame::Ptr& operator[](int index);
  const PreprocessedFrame::ConstPtr operator[](int index) const;

  void set_raw_points(const BatchedRawPoints::Ptr& batched_raw_points);

private:
  std::vector<PreprocessedFrame::Ptr> frames;
};

}  // namespace glim