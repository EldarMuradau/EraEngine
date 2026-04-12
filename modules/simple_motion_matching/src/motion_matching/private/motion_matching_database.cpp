#include "motion_matching/motion_matching_database.h"
#include "motion_matching/motion_matching_knn_structure.h"
#include "motion_matching/common.h"
#include "motion_matching/hnsw/hnsw_knn_structure.h"

#include <animation/animation_clip.h>

#include <asset/io.h>
#include <core/io/binary_data.h>
#include <core/io/binary_data_archive.h>

#include <core/math.h>
#include <core/log.h>

#include <core/traits.h>

namespace era_engine
{
    class PCAResult
    {
    public:
        PCAResult(uint32 num_components)
        {
            eigenvalues.resize(num_components);
            eigenvectors.resize(num_components);
        }

        std::vector<float> eigenvalues;
        std::vector<std::vector<float>> eigenvectors;
    };

    static float length(const std::vector<float>& query)
    {
        float sum = std::accumulate(query.begin(), query.end(), 0.0f, [](float sum, float v)
            {
                return sum + squaref(v);
            });

        return std::sqrt(sum);
    }

    static void normalize(std::vector<float>& query)
    {
        float l = length(query);
        if (fuzzy_equals(l, 0.0f))
        {
            return;
        }

        float normalizer = 1.0f / l;
        for (size_t index = 0; index < query.size(); ++index)
        {
            query[index] *= normalizer;
        }
    }

    static float square_distance(const std::vector<float>& lhs, const std::vector<float>& rhs)
    {
        ASSERT(lhs.size() == rhs.size());

        float result = 0.0f;
        for (size_t index = 0; index < lhs.size(); ++index)
        {
            const float diff = lhs[index] - rhs[index];
            result += squaref(diff);
        }

        return result;
    }

    static void eigen_decomposition_symmetric(array2d<float>& A, 
        std::vector<float>& eigenvalues,
        array2d<float>& eigenvectors, 
        uint32 max_iterations = 100)
    {
        uint32 n = A.rows;
        eigenvectors = array2d<float>(n, n);

        for (uint32 i = 0; i < n; ++i)
        {
            for (uint32 j = 0; j < n; ++j)
            {
                eigenvectors(i, j) = (i == j) ? 1.0f : 0.0f;
            }
        }

        eigenvalues.resize(n);

        for (uint32 iter = 0; iter < max_iterations; ++iter)
        {
            uint32 p = 0, q = 1;
            float max_off_diag = std::abs(A(0, 1));

            for (uint32 i = 0; i < n; ++i)
            {
                for (uint32 j = i + 1; j < n; ++j)
                {
                    float val = std::abs(A(i, j));
                    if (val > max_off_diag)
                    {
                        max_off_diag = val;
                        p = i;
                        q = j;
                    }
                }
            }

            if (max_off_diag < 1e-10f)
            {
                break;
            }

            float app = A(p, p);
            float aqq = A(q, q);
            float apq = A(p, q);

            float theta = 0.5f * std::atan2(2.0f * apq, aqq - app);
            float c = std::cos(theta);
            float s = std::sin(theta);

            for (uint32 i = 0; i < n; ++i)
            {
                if (i != p && i != q)
                {
                    float aip = A(i, p);
                    float aiq = A(i, q);
                    A(i, p) = c * aip - s * aiq;
                    A(p, i) = A(i, p);
                    A(i, q) = c * aiq + s * aip;
                    A(q, i) = A(i, q);
                }
            }

            float app_new = c * c * app - 2 * c * s * apq + s * s * aqq;
            float aqq_new = s * s * app + 2 * c * s * apq + c * c * aqq;
            A(p, p) = app_new;
            A(q, q) = aqq_new;
            A(p, q) = A(q, p) = 0.0f;

            for (uint32 i = 0; i < n; ++i)
            {
                float vip = eigenvectors(i, p);
                float viq = eigenvectors(i, q);
                eigenvectors(i, p) = c * vip - s * viq;
                eigenvectors(i, q) = c * viq + s * vip;
            }
        }

        for (uint32 i = 0; i < n; ++i)
        {
            eigenvalues[i] = A(i, i);
        }
    }

    static void pcabuildbasis(const array2d<float>& source_data, PCAResult& result)
    {
        uint32 num_samples = source_data.rows;
        uint32 num_features = source_data.cols;

        std::vector<float> means(num_features, 0.0f);

        for (uint32 j = 0; j < num_features; ++j)
        {
            for (uint32 i = 0; i < num_samples; ++i)
            {
                means[j] += source_data(i, j);
            }
            means[j] /= num_samples;
        }

        array2d<float> centered_data(num_samples, num_features);
        for (uint32 i = 0; i < num_samples; ++i)
        {
            for (uint32 j = 0; j < num_features; ++j)
            {
                centered_data(i, j) = source_data(i, j) - means[j];
            }
        }

        array2d<float> covariance(num_features, num_features);

        for (uint32 i = 0; i < num_features; ++i)
        {
            for (uint32 j = i; j < num_features; ++j)
            {
                float cov = 0.0f;
                for (uint32 k = 0; k < num_samples; ++k)
                {
                    cov += centered_data(k, i) * centered_data(k, j);
                }
                cov /= (num_samples - 1);
                covariance(i, j) = cov;
                covariance(j, i) = cov;
            }
        }

        std::vector<float> eigenvalues;
        array2d<float> eigenvectors(num_features, num_features);

        eigen_decomposition_symmetric(covariance, eigenvalues, eigenvectors);

        std::vector<uint32> indices(num_features);
        for (uint32 i = 0; i < num_features; ++i) indices[i] = i;

        std::sort(indices.begin(), indices.end(), [&](uint32 a, uint32 b)
            {
            return eigenvalues[a] > eigenvalues[b];
            });

        result = PCAResult(num_features);

        for (uint32 i = 0; i < num_features; ++i)
        {
            uint32 idx = indices[i];

            result.eigenvalues[i] = eigenvalues[idx];
            result.eigenvectors[i].resize(num_features);

            for (uint32 j = 0; j < num_features; ++j)
            {
                result.eigenvectors[i][j] = eigenvectors(j, idx);
            }
        }
    }

    void pcabuildbasis(const float* source_data_ptr, uint32 rows, uint32 cols,
        std::vector<float>& out_s, std::vector<std::vector<float>>& out_v)
    {
        array2d<float> mat(rows, cols);
        for (uint32 i = 0; i < rows * cols; ++i)
        {
            mat.data[i] = source_data_ptr[i];
        }

        PCAResult result(cols);
        pcabuildbasis(mat, result);

        out_s = result.eigenvalues;
        out_v = result.eigenvectors;
    }

    static array2d<float> build_transform_matrix(array2d<float> mat)
    {
        PCAResult result_pca(mat.cols);

        pcabuildbasis(mat, result_pca);

        const uint32 rows = result_pca.eigenvectors.size();
        const uint32 cols = result_pca.eigenvectors[0].size();

        array2d<float> result(rows, cols);
        for (uint32 row = 0; row < rows; ++row)
        {
            for (uint32 column = 0; column < cols; ++column)
            {
                result(row, column) = result_pca.eigenvectors[row][column];
            }
        }

        return result;
    }

    static std::vector<float> build_column_means(array2d<float>& mat)
    {
        std::vector<float> result;
        result.resize(mat.cols);
        for (int32 column = 0; column < mat.cols; ++column)
        {
            float sum = 0.0f;
            for (int32 row = 0; row < mat.rows; ++row)
            {
                sum += mat(row, column);
            }

            result[column] = sum / static_cast<float>(mat.rows);
        }

        return result;
    }

    static void substract_column_means(array2d<float>& mat, const std::vector<float>& column_means)
    {
        for (int32 column = 0; column < mat.cols; ++column)
        {
            float mean_value = column_means[column];
            for (int32 row = 0; row < mat.rows; ++row)
            {
                mat(row, column) -= mean_value;
            }
        }
    }

    static void separate(array2d<float>& mat, array2d<float>& output1, array2d<float>& output2, uint32 output1_column_count)
    {
        uint32 column_count1 = output1_column_count;
        uint32 column_count2 = mat.cols - output1_column_count;

        output1 = array2d<float>(mat.rows, column_count1);
        output2 = array2d<float>(mat.rows, column_count2);

        for (int32 row = 0; row < mat.rows; ++row)
        {
            for (int32 column = 0; column < mat.cols; ++column)
            {
                if (column < output1_column_count)
                {
                    output1(row, column) = mat(row, column);
                }
                else
                {
                    output2(row, column - output1_column_count) = mat(row, column);
                }
            }
        }
    }

	void MotionMatchingDatabase::fill_normalize_factors()
	{
        normalize_factors.clear();
		normalize_factors.reserve(max_values.size());
		for (size_t index = 0; index < max_values.size(); ++index)
		{
			if (fuzzy_equals(max_values[index], min_values[index], 1e-3f))
			{
				normalize_factors.push_back(0.0f);
			}
			else
			{
				normalize_factors.push_back(weights[index] / (max_values[index] - min_values[index]));
			}
		}
	}

    SearchResult MotionMatchingDatabase::narrow_phase_search(const std::vector<std::shared_ptr<Sample>>& broadphase_candidates,
        const std::vector<float>& normalized_query,
        const SearchParams& params) const
    {
        std::vector<std::shared_ptr<Sample>> resulted_candidates = broadphase_candidates;

        if (has_flag(narrow_phase_params.flags, NarrowPhaseFlags::STATIC_THRESHOLDS_CHECK))
        {
            for (auto iter = resulted_candidates.begin(); iter != resulted_candidates.end();)
            {
                const std::vector<float> candidates_features = iter->get()->features;

                if (candidates_features.size() != params.query.size())
                {
                    ASSERT(false);
                    iter = resulted_candidates.erase(iter);
                    continue;
                }

                bool is_valid_by_threshold = true;
                for (uint32 i = 0; i < params.query.size(); ++i)
                {
                    const StaticThreshold& threshold = static_thresholds[i];
                    if (!threshold.enabled)
                    {
                        continue;
                    }

                    const float diff = params.query[i] - candidates_features[i];

                    if (diff > threshold.max_diff ||
                        diff < threshold.min_diff)
                    {
                        is_valid_by_threshold = false;
                        break;
                    }
                }

                if (!is_valid_by_threshold)
                {
                    iter = resulted_candidates.erase(iter);
                    continue;
                }

                ++iter;
            }
        }

        if (resulted_candidates.empty())
        {
            return SearchResult();
        }

        if (has_flag(narrow_phase_params.flags, NarrowPhaseFlags::EUCLIDIAN_DISTANCE_CHECK))
        {
            for (auto iter = resulted_candidates.begin(); iter != resulted_candidates.end();)
            {
                const float result_candidate_to_query_distance_sqr = square_distance(params.query, normalize_query(iter->get()->features));

                const float max_euclidean_distance_sqr = squaref(narrow_phase_params.max_euclidean_distance);
                if (result_candidate_to_query_distance_sqr >= max_euclidean_distance_sqr)
                {
                    iter = resulted_candidates.erase(iter);
                    continue;
                }
            }
        }

        if (resulted_candidates.empty())
        {
            return SearchResult();
        }

        const std::shared_ptr<MotionMatchingDatabase::Sample>& result_candidate = resulted_candidates.front();

        if (has_flag(narrow_phase_params.flags, NarrowPhaseFlags::SAME_FRAME_CHECK) &&
            params.current_animation == animations[result_candidate->anim_index])
        {
            const float anim_position_diff = result_candidate->anim_position - params.current_anim_position;

            if (abs(anim_position_diff) < narrow_phase_params.same_frame_time_threshold)
			{
				return SearchResult(params.current_animation, database_id, params.current_features, params.current_anim_position);
			}
		}

		return SearchResult(animations[result_candidate->anim_index], database_id, result_candidate->features, result_candidate->anim_position);
	}

	MotionMatchingDatabase::MotionMatchingDatabase(KnnStructureType _knn_type /*= KnnStructureType::DEFAULT*/)
    {
        knn_type = _knn_type;
        if (knn_type == KnnStructureType::HNSW)
        {
            knn_structure = ref<KnnStructure>(new HnswKnnStructure());
        }
        else
        {
            ASSERT(false);
        }
    }

    MotionMatchingDatabase::~MotionMatchingDatabase()
    {
        for (MotionMatchingFeature* feature : features)
        {
            delete feature;
        }
    }

    void MotionMatchingDatabase::bake()
    {
        uint32 total_features_size = samples[0]->features.size();

        for (uint32 i = 0; i < total_features_size; i++)
        {
            min_values[i] = std::numeric_limits<float>::max();
            max_values[i] = std::numeric_limits<float>::min();
        }

        std::vector<float> feature_sums;
        feature_sums.resize(total_features_size, 0.0f);

        for (std::shared_ptr<Sample> motion_sample : samples)
        {
            ASSERT(motion_sample->features.size() == total_features_size);

            for (uint32 i = 0; i < total_features_size; i++)
            {
                if (motion_sample->features[i] < min_values[i])
                {
                    min_values[i] = motion_sample->features[i];
                }

                if (motion_sample->features[i] > max_values[i])
                {
                    max_values[i] = motion_sample->features[i];
                }

                feature_sums[i] += motion_sample->features[i];
            }
        }

        mean_values.clear();
        mean_values.reserve(total_features_size);
        for (uint32 i = 0; i < total_features_size; ++i)
        {
            mean_values.emplace_back(feature_sums[i] / samples.size());
        }

        fill_normalize_factors();

        std::vector<float> all_feature_values;
        all_feature_values.reserve(samples.size() * total_features_size);
		for (const auto& sample : samples)
		{
			for (uint32 index = 0; index < uint32(sample->features.size()); ++index)
			{
				float normalized_feature = 0.0f;

				if (fuzzy_equals(sample->features[index], min_values[index], 1e-3f))
				{
					normalized_feature = 0.0f;
				}
				else
				{
					float normalize_factor = weights[index] / (max_values[index] - min_values[index]);
					normalized_feature = (sample->features[index] - min_values[index]) * normalize_factor;
				}

				all_feature_values.push_back(normalized_feature);
			}
		}


        uint32 matching_row_count = uint32(samples.size());
        uint32 matching_column_count = uint32(total_features_size);

        array2d<float> features_matrix(matching_row_count, matching_column_count);
        features_matrix.fill(all_feature_values.data());

        array2d<float> pca_matrix = build_transform_matrix(features_matrix);
        transform_column_means = build_column_means(features_matrix);
        substract_column_means(features_matrix, transform_column_means);

        array2d<float> other;
        separate(pca_matrix, transform_matrix, other, search_dimension);
        transform_matrix = transpose(transform_matrix);
    }

    SearchResult MotionMatchingDatabase::search(const SearchParams& params) const
    {
        std::vector<float> normalized_query_values = normalize_query(params.query);
        const array2d<float> packed_matrix_query = pack_query(normalized_query_values);

        float* packed_normalized_query = packed_matrix_query.data;
        const uint32 query_size = packed_matrix_query.cols * packed_matrix_query.rows;

        ASSERT(query_size == search_dimension);

        std::vector<std::shared_ptr<Sample>> knn = knn_structure->search_knn(packed_normalized_query, query_size, max_broardphase_candidates, *this);

        return narrow_phase_search(knn, normalized_query_values, params);
    }

    std::vector<float> MotionMatchingDatabase::normalize_query(const std::vector<float>& query) const
    {
        ASSERT(query.size() == mean_values.size());
        ASSERT(query.size() == normalize_factors.size());

        std::vector<float> result;
        result.reserve(query.size());
        for (size_t index = 0; index < query.size(); ++index)
        {
            if (fuzzy_equals(query[index], mean_values[index], 1e-3f))
            {
                result.push_back(0.0f);
            }
            else
            {
                result.push_back((query[index] - mean_values[index]) * normalize_factors[index]);
            }
        }

        return result;
    }

    void MotionMatchingDatabase::subtract_column_means(array2d<float>& matrix) const
    {
        const uint32 rows = matrix.rows;
        const uint32 columns = matrix.cols;

        for (uint32 i = 0; i < rows; ++i)
        {
            for (uint32 j = 0; j < columns; ++j)
            {
                matrix(i, j) = matrix(i, j) - transform_column_means[j];
            }
        }
    }

    array2d<float> MotionMatchingDatabase::pack_query(const std::vector<float>& query) const
    {
        array2d<float> pack_target(1, uint32(query.size()));
        pack_target.fill(query.data());

        subtract_column_means(pack_target);

        return transpose(transform_matrix * transpose(pack_target));
    }

    std::string MotionMatchingDatabase::get_asset_type_impl()
    {
        return std::string(".emmdb");
    }

    bool MotionMatchingDatabase::serialize(std::ostream& os) const
    {
        try
        {
            // Base
            {
                IO::write_value(os, search_dimension);
                IO::write_value(os, sample_rate);
                IO::write_value(os, max_broardphase_candidates);
            }

            // Weights
            {
                IO::write_vector(os, normalize_factors);
                IO::write_vector(os, mean_values);
                IO::write_vector(os, min_values);
                IO::write_vector(os, max_values);
                IO::write_vector(os, weights);
            }

            // Animations
            {
                IO::write_vector(os, animations_asset_handles);
                IO::write_vector(os, animations_hashes);
            }

            // Features
            {
                IO::write_value(os, BinarySerializer::serialize(feature_types));
            }

            // Narrow phase
            {
                IO::write_value(os, narrow_phase_params.flags);
                IO::write_value(os, narrow_phase_params.same_frame_time_threshold);
                IO::write_value(os, narrow_phase_params.max_euclidean_distance);

                IO::write_value(os, BinarySerializer::serialize(static_thresholds));
            }

            // Transform
            {
                IO::write_vector(os, transform_column_means);
                IO::write_value(os, transform_matrix.cols);
                IO::write_value(os, transform_matrix.rows);
                IO::write_data(os, transform_matrix.data, static_cast<uint32>(transform_matrix.cols * transform_matrix.rows));
            }

            // Knn
            {
                IO::write_value(os, knn_type);

                if (knn_structure->writable.empty())
                {
                    knn_structure->build_structure(*this);
                }

                IO::write_value(os, knn_structure->writable.size());
                IO::write_data(os, knn_structure->writable.data(), knn_structure->writable.size());

                knn_structure->serialize(os, *this);
            }
        }
        catch (...)
        {
            LOG_ERROR("Exception thrown while serializing asset!");
            return false;
        }

        return true;
    }

    bool MotionMatchingDatabase::deserialize(std::istream& is)
    {
        try
        {
            // Base
            {
                IO::read_value(is, search_dimension);
                IO::read_value(is, sample_rate);
                IO::read_value(is, max_broardphase_candidates);
            }

            // Weights
            {
                IO::read_vector(is, normalize_factors);
                IO::read_vector(is, mean_values);
                IO::read_vector(is, min_values);
                IO::read_vector(is, max_values);
                IO::read_vector(is, weights);
            }

            // Animations
            {
                IO::read_vector(is, animations_asset_handles);
                IO::read_vector(is, animations_hashes);

                GameAssetsProvider provider;

                for (const uint64 handle : animations_asset_handles)
                {
                    ref<animation::AnimationAssetClip> anim_clip = provider.load_game_asset_from_file<animation::AnimationAssetClip>(getPathFromAssetHandle(AssetHandle(handle)));
                    animations.emplace_back(anim_clip);
                }
            }

            // Features
            {
                BinaryDataArchive binary_feature_names;
                IO::read_value(is, binary_feature_names);

                std::vector<std::string> feature_serialized_names;
                BinarySerializer::deserialize(binary_feature_names.raw_data(), binary_feature_names.size(), feature_serialized_names);

                feature_types.reserve(feature_serialized_names.size());
                for (const std::string& param_name : feature_serialized_names)
                {
                    feature_types.emplace_back(param_name);

                    rttr::type feature_type = rttr::type::get_by_name(param_name);
                    features.emplace_back(feature_type.create().get_value<MotionMatchingFeature*>());
                }
            }

            // Narrow phase
            {
                IO::read_value(is, narrow_phase_params.flags);
                IO::read_value(is, narrow_phase_params.same_frame_time_threshold);
                IO::read_value(is, narrow_phase_params.max_euclidean_distance);

                BinaryDataArchive binary_static_thresholds;
                IO::read_value(is, binary_static_thresholds);

                BinarySerializer::deserialize(binary_static_thresholds.raw_data(), binary_static_thresholds.size(), static_thresholds);
            }

            // Transform
            {
                IO::read_vector(is, transform_column_means);
                IO::read_value(is, transform_matrix.cols);
                IO::read_value(is, transform_matrix.rows);
                IO::read_data(is, transform_matrix.data, static_cast<uint32>(transform_matrix.cols * transform_matrix.rows));
            }

            // Knn
            {
                IO::read_value(is, knn_type);

                uint32 knn_size = 0;
                IO::read_value(is, knn_size);

                std::vector<uint8> knn_data;
                knn_data.reserve(knn_size);
                IO::read_data(is, knn_data.data(), knn_size);

                knn_structure->writable.assign(
                    reinterpret_cast<const char*>(knn_data.data()),
                    knn_size);

                knn_structure->deserialize(is, *this);
            }

            // Wait for loading
            for (const auto& animation : animations)
            {
                animation->load_job.wait_for_completion();
            }
        }
        catch (...)
        {
            LOG_ERROR("Exception thrown while deserializing asset!");
            return false;
        }

        return true;
    }

    static std::unordered_map<std::string, ref<MotionMatchingDatabase>> databases_in_registry;
    static std::mutex database_registry_mtx;

    void MotionDatabaseRegistry::add_to_registry(ref<MotionMatchingDatabase> database)
    {
        if (databases_in_registry.count(database->database_id) <= 0)
        {
            std::lock_guard<std::mutex> lock{ database_registry_mtx };
            databases_in_registry.emplace(database->database_id, database);
        }
    }

    ref<MotionMatchingDatabase> MotionDatabaseRegistry::get_by_id(const std::string& database_id)
    {
        auto database_iter = databases_in_registry.find(database_id);
        if (database_iter == databases_in_registry.end())
        {
            return nullptr;
        }
        return database_iter->second;
    }
}
