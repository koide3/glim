#include <glim/mapping/graph_metadata.hpp>

#include <algorithm>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace glim {
namespace {

template <typename... Args>
void parse_record(std::istringstream& stream, const int line_number, Args&... args) {
  if (!(stream >> ... >> args)) {
    throw std::runtime_error("truncated graph metadata record at line " + std::to_string(line_number));
  }

  std::string extra;
  if (stream >> extra) {
    throw std::runtime_error("unexpected value in graph metadata record at line " + std::to_string(line_number));
  }
}

void set_once(bool& seen, const std::string& name, const int line_number) {
  if (seen) {
    throw std::runtime_error("duplicate " + name + " record at line " + std::to_string(line_number));
  }
  seen = true;
}

void validate_nonnegative(const int value, const std::string& name) {
  if (value < 0) {
    throw std::runtime_error(name + " must be nonnegative");
  }
}

}  // namespace

std::vector<SubmapRange> normalize_submap_ranges(const std::vector<SubmapRange>& ranges) {
  std::vector<SubmapRange> normalized = ranges;
  for (const auto& range : normalized) {
    if (range.first < 0 || range.first > range.last) {
      throw std::invalid_argument("invalid submap range [" + std::to_string(range.first) + ", " + std::to_string(range.last) + "]");
    }
  }

  std::sort(normalized.begin(), normalized.end(), [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first || (lhs.first == rhs.first && lhs.last < rhs.last); });

  std::vector<SubmapRange> merged;
  for (const auto& range : normalized) {
    if (merged.empty() || static_cast<long long>(range.first) > static_cast<long long>(merged.back().last) + 1) {
      merged.push_back(range);
    } else {
      merged.back().last = std::max(merged.back().last, range.last);
    }
  }

  return merged;
}

std::vector<SubmapRange> union_submap_ranges(const std::vector<SubmapRange>& lhs, const std::vector<SubmapRange>& rhs) {
  std::vector<SubmapRange> combined = lhs;
  combined.insert(combined.end(), rhs.begin(), rhs.end());
  return normalize_submap_ranges(combined);
}

std::vector<SubmapRange> offset_submap_ranges(const std::vector<SubmapRange>& ranges, const int offset) {
  auto shifted = normalize_submap_ranges(ranges);
  for (auto& range : shifted) {
    const long long first = static_cast<long long>(range.first) + offset;
    const long long last = static_cast<long long>(range.last) + offset;
    if (first < 0 || last > std::numeric_limits<int>::max()) {
      throw std::out_of_range("submap range offset is out of bounds");
    }
    range.first = static_cast<int>(first);
    range.last = static_cast<int>(last);
  }
  return shifted;
}

std::vector<uint8_t> submap_ranges_to_mask(const std::vector<SubmapRange>& ranges, const int num_submaps) {
  if (num_submaps < 0) {
    throw std::invalid_argument("num_submaps must be nonnegative");
  }

  std::vector<uint8_t> mask(num_submaps, 0);
  for (const auto& range : normalize_submap_ranges(ranges)) {
    if (range.last >= num_submaps) {
      throw std::out_of_range("submap range exceeds num_submaps");
    }
    std::fill(mask.begin() + range.first, mask.begin() + range.last + 1, 1);
  }
  return mask;
}

std::vector<int> submap_ranges_to_ids(const std::vector<SubmapRange>& ranges) {
  std::vector<int> ids;
  for (const auto& range : normalize_submap_ranges(ranges)) {
    for (int id = range.first; id <= range.last; id++) {
      ids.push_back(id);
    }
  }
  return ids;
}

std::vector<SubmapRange> submap_ids_to_ranges(const std::vector<int>& ids) {
  std::vector<int> normalized_ids = ids;
  for (const int id : normalized_ids) {
    if (id < 0) {
      throw std::invalid_argument("submap ID must be nonnegative");
    }
  }

  std::sort(normalized_ids.begin(), normalized_ids.end());
  normalized_ids.erase(std::unique(normalized_ids.begin(), normalized_ids.end()), normalized_ids.end());

  std::vector<SubmapRange> ranges;
  for (const int id : normalized_ids) {
    if (ranges.empty() || static_cast<long long>(id) > static_cast<long long>(ranges.back().last) + 1) {
      ranges.push_back({id, id});
    } else {
      ranges.back().last = id;
    }
  }
  return ranges;
}

int count_submaps(const std::vector<SubmapRange>& ranges) {
  int count = 0;
  for (const auto& range : normalize_submap_ranges(ranges)) {
    count += range.last - range.first + 1;
  }
  return count;
}

bool is_submap_pruned(const std::vector<uint8_t>& mask, const int submap_id) {
  return submap_id >= 0 && submap_id < static_cast<int>(mask.size()) && mask[submap_id] != 0;
}

GraphMetadata parse_graph_metadata(std::istream& stream) {
  struct RequiredRecords {
    bool num_submaps = false;
    bool num_all_frames = false;
    bool num_active_frames = false;
    bool num_matching_cost_factors = false;
    bool num_pruned_submaps = false;
    bool num_pruned_ranges = false;
  } seen;

  GraphMetadata metadata;
  int declared_matching_cost_factors = -1;
  int declared_pruned_submaps = -1;
  int declared_pruned_ranges = -1;

  std::string line;
  int line_number = 0;
  bool has_records = false;
  while (std::getline(stream, line)) {
    line_number++;
    std::istringstream line_stream(line);
    std::string record;
    if (!(line_stream >> record)) {
      continue;
    }
    has_records = true;

    if (record == "num_submaps:") {
      set_once(seen.num_submaps, record, line_number);
      parse_record(line_stream, line_number, metadata.num_submaps);
    } else if (record == "num_all_frames:") {
      set_once(seen.num_all_frames, record, line_number);
      parse_record(line_stream, line_number, metadata.num_all_frames);
    } else if (record == "num_active_frames:") {
      set_once(seen.num_active_frames, record, line_number);
      parse_record(line_stream, line_number, metadata.num_active_frames);
    } else if (record == "num_matching_cost_factors:") {
      set_once(seen.num_matching_cost_factors, record, line_number);
      parse_record(line_stream, line_number, declared_matching_cost_factors);
    } else if (record == "num_pruned_submaps:") {
      set_once(seen.num_pruned_submaps, record, line_number);
      parse_record(line_stream, line_number, declared_pruned_submaps);
    } else if (record == "num_pruned_ranges:") {
      set_once(seen.num_pruned_ranges, record, line_number);
      parse_record(line_stream, line_number, declared_pruned_ranges);
    } else if (record == "pruned_range") {
      SubmapRange range;
      parse_record(line_stream, line_number, range.first, range.last);
      metadata.pruned_ranges.push_back(range);
    } else if (record == "matching_cost") {
      MatchingCostRecord factor;
      parse_record(line_stream, line_number, factor.type, factor.first, factor.second);
      metadata.matching_cost_factors.push_back(factor);
    } else {
      continue;
    }
  }

  if (!has_records) {
    throw std::runtime_error("graph metadata is empty");
  }
  if (!seen.num_submaps || !seen.num_all_frames || !seen.num_matching_cost_factors) {
    throw std::runtime_error("graph metadata is missing a required count record");
  }

  validate_nonnegative(metadata.num_submaps, "num_submaps");
  validate_nonnegative(metadata.num_all_frames, "num_all_frames");
  validate_nonnegative(declared_matching_cost_factors, "num_matching_cost_factors");
  if (declared_matching_cost_factors != static_cast<int>(metadata.matching_cost_factors.size())) {
    throw std::runtime_error("num_matching_cost_factors does not match matching_cost records");
  }

  const bool has_pruning_records = seen.num_active_frames || seen.num_pruned_submaps || seen.num_pruned_ranges || !metadata.pruned_ranges.empty();
  if (!has_pruning_records) {
    metadata.num_active_frames = metadata.num_all_frames;
  } else {
    if (!seen.num_active_frames || !seen.num_pruned_submaps || !seen.num_pruned_ranges) {
      throw std::runtime_error("graph metadata is missing a required pruning count record");
    }
    validate_nonnegative(metadata.num_active_frames, "num_active_frames");
    validate_nonnegative(declared_pruned_submaps, "num_pruned_submaps");
    validate_nonnegative(declared_pruned_ranges, "num_pruned_ranges");
    if (metadata.num_active_frames > metadata.num_all_frames) {
      throw std::runtime_error("num_active_frames exceeds num_all_frames");
    }
    if (declared_pruned_ranges != static_cast<int>(metadata.pruned_ranges.size())) {
      throw std::runtime_error("num_pruned_ranges does not match pruned_range records");
    }

    const auto normalized = normalize_submap_ranges(metadata.pruned_ranges);
    if (normalized != metadata.pruned_ranges) {
      throw std::runtime_error("pruned_range records are not canonical");
    }
    if (count_submaps(metadata.pruned_ranges) != declared_pruned_submaps) {
      throw std::runtime_error("num_pruned_submaps does not match pruned_range records");
    }
  }

  const auto pruned_mask = submap_ranges_to_mask(metadata.pruned_ranges, metadata.num_submaps);
  for (const auto& factor : metadata.matching_cost_factors) {
    if (factor.first < 0 || factor.first >= metadata.num_submaps || factor.second < 0 || factor.second >= metadata.num_submaps) {
      throw std::runtime_error("matching_cost references a missing submap");
    }
    if (is_submap_pruned(pruned_mask, factor.first) || is_submap_pruned(pruned_mask, factor.second)) {
      throw std::runtime_error("matching_cost references a pruned submap");
    }
  }

  return metadata;
}

void write_graph_metadata(std::ostream& stream, const GraphMetadata& metadata) {
  if (metadata.num_submaps < 0 || metadata.num_all_frames < 0 || metadata.num_active_frames < 0 || metadata.num_active_frames > metadata.num_all_frames) {
    throw std::invalid_argument("invalid graph metadata counts");
  }

  const auto pruned_ranges = normalize_submap_ranges(metadata.pruned_ranges);
  const auto pruned_mask = submap_ranges_to_mask(pruned_ranges, metadata.num_submaps);
  for (const auto& factor : metadata.matching_cost_factors) {
    if (
      factor.first < 0 || factor.first >= metadata.num_submaps || factor.second < 0 || factor.second >= metadata.num_submaps || is_submap_pruned(pruned_mask, factor.first) ||
      is_submap_pruned(pruned_mask, factor.second)) {
      throw std::invalid_argument("matching_cost references a missing or pruned submap");
    }
  }

  stream << "num_submaps: " << metadata.num_submaps << '\n';
  stream << "num_all_frames: " << metadata.num_all_frames << '\n';
  stream << "num_matching_cost_factors: " << metadata.matching_cost_factors.size() << '\n';
  for (const auto& factor : metadata.matching_cost_factors) {
    stream << "matching_cost " << factor.type << ' ' << factor.first << ' ' << factor.second << '\n';
  }
  stream << "num_active_frames: " << metadata.num_active_frames << '\n';
  stream << "num_pruned_submaps: " << count_submaps(pruned_ranges) << '\n';
  stream << "num_pruned_ranges: " << pruned_ranges.size() << '\n';
  for (const auto& range : pruned_ranges) {
    stream << "pruned_range " << range.first << ' ' << range.last << '\n';
  }

  if (!stream) {
    throw std::runtime_error("failed to write graph metadata");
  }
}

}  // namespace glim
