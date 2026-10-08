#pragma once

#include <cstdint>
#include <iosfwd>
#include <string>
#include <vector>

namespace glim {

/**
 * @brief Inclusive submap ID range
 */
struct SubmapRange {
  int first;  ///< First submap ID
  int last;   ///< Last submap ID (inclusive)

  bool operator==(const SubmapRange& other) const { return first == other.first && last == other.last; }
  bool operator!=(const SubmapRange& other) const { return !(*this == other); }
};

/**
 * @brief Sort and merge overlapping or adjacent submap ranges
 * @param ranges Input ranges
 * @return Canonical inclusive ranges
 */
std::vector<SubmapRange> normalize_submap_ranges(const std::vector<SubmapRange>& ranges);

/**
 * @brief Compute the union of two submap range sets
 * @param lhs First range set
 * @param rhs Second range set
 * @return Canonical union
 */
std::vector<SubmapRange> union_submap_ranges(const std::vector<SubmapRange>& lhs, const std::vector<SubmapRange>& rhs);

/**
 * @brief Shift all submap IDs by a fixed offset
 * @param ranges Input ranges
 * @param offset ID offset
 * @return Shifted canonical ranges
 */
std::vector<SubmapRange> offset_submap_ranges(const std::vector<SubmapRange>& ranges, int offset);

/**
 * @brief Expand submap ranges into an index-aligned mask
 * @param ranges Input ranges
 * @param num_submaps Mask size
 * @return Byte mask with nonzero entries for selected submaps
 */
std::vector<uint8_t> submap_ranges_to_mask(const std::vector<SubmapRange>& ranges, int num_submaps);

/**
 * @brief Expand submap ranges into sorted IDs
 * @param ranges Input ranges
 * @return Sorted unique submap IDs
 */
std::vector<int> submap_ranges_to_ids(const std::vector<SubmapRange>& ranges);

/**
 * @brief Convert submap IDs to canonical ranges
 * @param ids Input submap IDs
 * @return Canonical inclusive ranges
 */
std::vector<SubmapRange> submap_ids_to_ranges(const std::vector<int>& ids);

/**
 * @brief Count unique submaps represented by ranges
 * @param ranges Input ranges
 * @return Number of represented submaps
 */
int count_submaps(const std::vector<SubmapRange>& ranges);

/**
 * @brief Test a submap ID against an index-aligned pruned mask
 * @param mask Pruned submap mask
 * @param submap_id Submap ID
 * @return True if the submap is marked as pruned
 */
bool is_submap_pruned(const std::vector<uint8_t>& mask, int submap_id);

/**
 * @brief Serialized matching-cost factor descriptor
 */
struct MatchingCostRecord {
  std::string type;  ///< Matching factor type
  int first;         ///< First submap ID
  int second;        ///< Second submap ID

  bool operator==(const MatchingCostRecord& other) const { return type == other.type && first == other.first && second == other.second; }
};

/**
 * @brief Parsed graph.txt metadata
 */
struct GraphMetadata {
  int num_submaps = 0;                                    ///< Archived submaps, including pruned submaps
  int num_all_frames = 0;                                 ///< All archived frames
  int num_active_frames = 0;                              ///< Frames belonging to active submaps
  std::vector<SubmapRange> pruned_ranges;                 ///< Canonical pruned submap ranges
  std::vector<MatchingCostRecord> matching_cost_factors;  ///< Active matching-cost factor records
};

/**
 * @brief Parse graph metadata with optional pruning records
 * @param stream Input graph.txt stream
 * @return Validated graph metadata
 * @throws std::runtime_error if the metadata is malformed
 */
GraphMetadata parse_graph_metadata(std::istream& stream);

/**
 * @brief Write graph metadata
 * @param stream Output graph.txt stream
 * @param metadata Metadata to validate and write
 */
void write_graph_metadata(std::ostream& stream, const GraphMetadata& metadata);

}  // namespace glim
