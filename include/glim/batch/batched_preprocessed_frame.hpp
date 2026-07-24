#pragma once

#include <memory>
#include <glim/preprocess/preprocessed_frame.hpp>

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

private:
  std::vector<PreprocessedFrame::Ptr> frames;
};

}  // namespace glim