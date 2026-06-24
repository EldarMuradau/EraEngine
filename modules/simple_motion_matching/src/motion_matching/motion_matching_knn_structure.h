#pragma once 

#include "motion_matching_api.h"

#include "motion_matching/motion_matching_database.h"

namespace era_engine 
{
	class ERA_MOTION_MATCHING_API KnnStructure
	{
	public:
		virtual ~KnnStructure();

		virtual void rebuild_structure(const MotionMatchingDatabase& database);
		virtual void build_structure_from_matrix(const MotionMatchingDatabase& database, const array2d<float>& compressed_matrix);

		virtual std::vector<std::shared_ptr<MotionMatchingDatabase::Sample>> search_knn(float* query, uint32 query_size, uint32 max_candidates, const MotionMatchingDatabase& database);
	
		virtual bool serialize(std::ostream& os, const MotionMatchingDatabase& database) const;
		virtual bool deserialize(std::istream& is, const MotionMatchingDatabase& database);

	public:
		mutable std::string writable; // Serializable data
	};
}