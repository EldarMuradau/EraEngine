#include "motion_matching/hnsw/hnsw_knn_structure.h"

#include <core/math.h>

namespace era_engine
{
    HnswKnnStructure::~HnswKnnStructure()
    {
    }

    void HnswKnnStructure::build_structure(const MotionMatchingDatabase& database)
    {
        hnsw_l2_space = hnswlib::L2Space(database.search_dimension);

        std::vector<float> features_array;
        features_array.reserve(database.samples.size() * database.search_dimension);

        for (size_t sample_index = 0; sample_index < database.samples.size(); ++sample_index)
        {
            const std::shared_ptr<MotionMatchingDatabase::Sample>& sample = database.samples[sample_index];
            const std::vector<float> sample_features = database.normalize_query(sample->features);

            features_array.insert(features_array.end(), sample_features.begin(), sample_features.end());
        }

        array2d<float> features_matrix(uint32(database.samples.size()), database.samples.front()->features.size());
        features_matrix.fill(features_array.data());

        database.subtract_column_means(features_matrix);

        const array2d<float> final_features_matrix = transpose(database.transform_matrix * transpose(features_matrix));

        hnsw = std::shared_ptr<hnswlib::HierarchicalNSW<float>>(new hnswlib::HierarchicalNSW<float>(&hnsw_l2_space, database.samples.size(), max_edges_per_vertex, construction_exploration_factor));

        for (uint32 point_index = 0; point_index < static_cast<uint32>(database.samples.size()); ++point_index)
        {
            std::vector<float> animation_data;

            for (uint32 feature_index = 0; feature_index < database.search_dimension; ++feature_index)
            {
                animation_data.push_back(final_features_matrix(point_index, feature_index));
            }

            hnsw->addPoint(animation_data.data(), point_index);
        }

        hnsw->saveIndex(writable);
    }

    std::vector<std::shared_ptr<MotionMatchingDatabase::Sample>> HnswKnnStructure::search_knn(float* query, uint32 query_size, uint32 max_candidates, const MotionMatchingDatabase& database)
    {
        std::vector<std::shared_ptr<MotionMatchingDatabase::Sample>> found_samples;

        std::priority_queue<std::pair<float, hnswlib::labeltype>> search_result = hnsw->searchKnn(query, max_candidates);

        const int32 size = int32(search_result.size());
        found_samples.reserve(size);

        for (int32 index = size - 1; index > 0; --index)
        {
            const int32 sample_index = int32(search_result.top().second);
            search_result.pop();

            ASSERT(sample_index < database.samples.size());
            found_samples[index] = database.samples[sample_index];
        }

        return found_samples;
    }

	bool HnswKnnStructure::serialize(std::ostream& os, const MotionMatchingDatabase& database) const
	{
        IO::write_value(os, max_edges_per_vertex);
        IO::write_value(os, construction_exploration_factor);

		return true;
	}

    bool HnswKnnStructure::deserialize(std::istream& is, const MotionMatchingDatabase& database)
    {
        IO::read_value(is, max_edges_per_vertex);
        IO::read_value(is, construction_exploration_factor);

        hnsw_l2_space = hnswlib::L2Space(database.search_dimension);

        if (writable.empty())
        {
            build_structure(database);
        }
        else
        {
            hnsw = std::shared_ptr<hnswlib::HierarchicalNSW<float>>(new hnswlib::HierarchicalNSW<float>(&hnsw_l2_space));

            hnsw->loadIndex(writable, &hnsw_l2_space);
        }
        return false;
    }
}
