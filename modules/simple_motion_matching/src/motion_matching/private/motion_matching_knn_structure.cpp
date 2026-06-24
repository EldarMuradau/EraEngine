#include "motion_matching/motion_matching_knn_structure.h"

#include <core/math.h>

namespace era_engine
{
    KnnStructure::~KnnStructure()
    {
    }

    void KnnStructure::rebuild_structure(const MotionMatchingDatabase& database)
    {
    }

    void KnnStructure::build_structure_from_matrix(const MotionMatchingDatabase& database, const array2d<float>& compressed_matrix)
    {
    }

    std::vector<std::shared_ptr<MotionMatchingDatabase::Sample>> KnnStructure::search_knn(float* query, uint32 query_size, uint32 max_candidates, const MotionMatchingDatabase& database)
    {
        return std::vector<std::shared_ptr<MotionMatchingDatabase::Sample>>();
    }

    bool KnnStructure::serialize(std::ostream& os, const MotionMatchingDatabase& database) const
    {
        return false;
    }

    bool KnnStructure::deserialize(std::istream& is, const MotionMatchingDatabase& database)
    {
        return false;
    }
}
