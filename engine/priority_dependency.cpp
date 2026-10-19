#include "priority_dependency.h"
#include "../constants/voxel_constants.h"
#include "../util/math/funcs.h"

namespace zylann::voxel {

TaskPriority PriorityDependency::evaluate(uint8_t lod_index, uint8_t band2_priority, float *out_closest_distance_sq) {
	TaskPriority priority;
	ZN_ASSERT_RETURN_V(shared != nullptr, priority);

	const StdVector<Vector3f> &viewer_positions = shared->viewers;
	const StdVector<unsigned int> &viewer_view_distances = shared->view_distances;
	const unsigned int viewer_count = shared->viewers_count;

	const Vector3f block_position = world_position;

	float closest_distance_sq = 99999.f;
	float closest_ratio = 1.f;
	if (viewer_positions.size() == 0) {
		closest_distance_sq = math::length_squared(block_position);
		closest_ratio = math::max(Math::sqrt(closest_distance_sq) - radius, 0.f) / float(0xffff);
	} else {
		for (unsigned int i = 0; i < viewer_count; ++i) {
			const float d = math::distance_squared(viewer_positions[i], block_position);
			closest_distance_sq = math::min(closest_distance_sq, d);
			closest_ratio = math::min(
					closest_ratio,
					math::max(Math::sqrt(d) - radius, 0.f) / float(math::max(viewer_view_distances[i], 1u))
			);
		}
	}

	if (out_closest_distance_sq != nullptr) {
		*out_closest_distance_sq = closest_distance_sq;
	}

	priority.band0 = constants::MAX_LOD - lod_index;
	priority.band1 = TaskPriority::BAND_MAX - math::min(static_cast<int>(Math::sqrt(closest_ratio) * TaskPriority::BAND_MAX), int(TaskPriority::BAND_MAX));
	priority.band2 = band2_priority;
	priority.band3 = constants::TASK_PRIORITY_BAND3_DEFAULT;

	return priority;
}

} // namespace zylann::voxel
