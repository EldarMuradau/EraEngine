#include "asset/model_importer.h"

#include "asset/model_asset.h"
#include "asset/asset.h"
#include "asset/io.h"

#include "animation/skeleton.h"
#include "animation/skinning.h"
#include "animation/animation_clip_utils.h"
#include "animation/animation.h"

#include "core/log.h"

#include <material.hlsli>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

namespace era_engine
{
    static void load_materials(const aiScene* scene, ModelAsset& result)
    {
        result.materials.reserve(scene->mNumMaterials);

        for (uint32 i = 0; i < scene->mNumMaterials; ++i)
        {
            aiMaterial* material = scene->mMaterials[i];
            PbrMaterialDesc imported;

            aiColor4D albedo;
            if (material->Get(AI_MATKEY_COLOR_DIFFUSE, albedo) == AI_SUCCESS)
            {
                imported.albedo_tint = vec4(albedo.r, albedo.g, albedo.b, albedo.a);
            }

            float roughness = 0.5f;
            if (material->Get(AI_MATKEY_ROUGHNESS_FACTOR, roughness) == AI_SUCCESS)
            {
                imported.roughness_override = roughness;
            }
            else
            {
                float shininess = 0.0f;
                if (material->Get(AI_MATKEY_SHININESS, shininess) == AI_SUCCESS)
                {
                    imported.roughness_override = 1.0f - sqrt(shininess / 100.0f);
                }
            }

            float metallic = 0.0f;
            material->Get(AI_MATKEY_METALLIC_FACTOR, metallic);
            imported.metallic_override = metallic;

            aiString texturePath;

            if (material->GetTexture(aiTextureType_DIFFUSE, 0, &texturePath) == AI_SUCCESS)
            {
                imported.albedo = texturePath.C_Str();
            }

            if (material->GetTexture(aiTextureType_NORMALS, 0, &texturePath) == AI_SUCCESS)
            {
                imported.normal = texturePath.C_Str();
            }

            if (material->GetTexture(aiTextureType_DIFFUSE_ROUGHNESS, 0, &texturePath) == AI_SUCCESS)
            {
                imported.roughness = texturePath.C_Str();
            }

            if (material->GetTexture(aiTextureType_METALNESS, 0, &texturePath) == AI_SUCCESS)
            {
                imported.metallic = texturePath.C_Str();
            }

            result.materials.push_back(imported);
        }
    }

    static void add_bone_weight(animation::SkinningWeights& skinning, uint32 boneIndex, float weight)
    {
        uint8 weightByte = (uint8)clamp(weight * 255.0f, 0.0f, 255.0f);
        uint8 boneID = (uint8)boneIndex;

        for (int i = 0; i < 4; ++i)
        {
            if (skinning.skin_weights[i] == 0)
            {
                skinning.skin_indices[i] = boneID;
                skinning.skin_weights[i] = weightByte;
                return;
            }
        }

        int minIndex = 0;
        uint8 minWeight = skinning.skin_weights[0];

        for (int i = 1; i < 4; ++i)
        {
            if (skinning.skin_weights[i] < minWeight)
            {
                minIndex = i;
                minWeight = skinning.skin_weights[i];
            }
        }

        if (weightByte > minWeight)
        {
            skinning.skin_indices[minIndex] = boneID;
            skinning.skin_weights[minIndex] = weightByte;
        }
    }


    static void import_skinning(aiMesh* aiMesh, SubmeshAsset& submesh)
    {
        submesh.skin.resize(aiMesh->mNumVertices);

        for (uint32 boneIndex = 0; boneIndex < aiMesh->mNumBones; ++boneIndex)
        {
            aiBone* bone = aiMesh->mBones[boneIndex];

            for (uint32 weightIndex = 0; weightIndex < bone->mNumWeights; ++weightIndex)
            {
                aiVertexWeight weight = bone->mWeights[weightIndex];

                uint32 vertexID = weight.mVertexId;
                float weightValue = weight.mWeight;

                if (vertexID >= submesh.skin.size())
                {
                    continue;
                }

                add_bone_weight(submesh.skin[vertexID], boneIndex, weightValue);
            }
        }
    }

    static void load_meshes(const aiScene* scene, float scaleFactor, ModelAsset& result)
    {
        result.meshes.reserve(scene->mNumMeshes);

        for (uint32 meshIndex = 0; meshIndex < scene->mNumMeshes; ++meshIndex)
        {
            aiMesh* aiMesh = scene->mMeshes[meshIndex];

            MeshAsset mesh;
            mesh.name = aiMesh->mName.C_Str();
            mesh.skeleton_index = 0;

            SubmeshAsset submesh;
            submesh.material_index = aiMesh->mMaterialIndex;

            submesh.positions.reserve(aiMesh->mNumVertices);
            for (uint32 i = 0; i < aiMesh->mNumVertices; ++i)
            {
                aiVector3D pos = aiMesh->mVertices[i];
                
                submesh.positions.push_back(vec3(pos.x, pos.y, pos.z) * scaleFactor);
            }

            if (aiMesh->HasNormals())
            {
                submesh.normals.reserve(aiMesh->mNumVertices);
                for (uint32 i = 0; i < aiMesh->mNumVertices; ++i)
                {
                    aiVector3D normal = aiMesh->mNormals[i];
                    submesh.normals.push_back(vec3(normal.x, normal.y, normal.z));
                }
            }

            if (aiMesh->HasTextureCoords(0))
            {
                submesh.uvs.reserve(aiMesh->mNumVertices);
                for (uint32 i = 0; i < aiMesh->mNumVertices; ++i)
                {
                    aiVector3D uv = aiMesh->mTextureCoords[0][i];
                    submesh.uvs.push_back(vec2(uv.x, uv.y));
                }
            }

            if (aiMesh->HasTangentsAndBitangents())
            {
                submesh.tangents.reserve(aiMesh->mNumVertices);
                for (uint32 i = 0; i < aiMesh->mNumVertices; ++i)
                {
                    aiVector3D tangent = aiMesh->mTangents[i];
                    submesh.tangents.push_back(vec3(tangent.x, tangent.y, tangent.z));
                }
            }

            if (aiMesh->HasVertexColors(0))
            {
                submesh.colors.reserve(aiMesh->mNumVertices);
                for (uint32 i = 0; i < aiMesh->mNumVertices; ++i)
                {
                    aiColor4D color = aiMesh->mColors[0][i];
                    uint32 packed = packColor(color.r, color.g, color.b, color.a);
                    submesh.colors.push_back(packed);
                }
            }

            if (aiMesh->HasBones())
            {
                import_skinning(aiMesh, submesh);
            }

            submesh.triangles.reserve(aiMesh->mNumFaces);

            for (uint32 faceIndex = 0; faceIndex < aiMesh->mNumFaces; ++faceIndex)
            {
                aiFace face = aiMesh->mFaces[faceIndex];

                if (face.mNumIndices == 3)
                {
                    indexed_triangle32 triangle;
                    triangle.a = (uint32)face.mIndices[0];
                    triangle.b = (uint32)face.mIndices[1];
                    triangle.c = (uint32)face.mIndices[2];
                    submesh.triangles.push_back(triangle);
                }
                else
                {
                    printf("Warning: Face with %d indices (expected 3)\n", face.mNumIndices);
                }
            }

            mesh.submeshes.push_back(submesh);
            result.meshes.push_back(mesh);
        }
    }
    static std::string clean_bone_name(const std::string& name)
    {
        size_t pos = name.find("_$AssimpFbx$");
        if (pos != std::string::npos)
        {
            return name.substr(0, pos);
        }
        return name;
    }

    static void find_skeleton_roots_recursive(aiNode* node,
        const std::unordered_set<std::string>& boneNames,
        std::vector<aiNode*>& roots)
    {
        std::string nodeName = clean_bone_name(node->mName.C_Str());

        bool isBone = boneNames.find(nodeName) != boneNames.end();

        if (isBone)
        {
            bool parentIsBone = false;

            if (node->mParent)
            {
                std::string parentName = clean_bone_name(node->mParent->mName.C_Str());
                parentIsBone = boneNames.find(parentName) != boneNames.end();
            }

            if (!parentIsBone)
            {
                roots.push_back(node);
                return;
            }
        }

        for (uint32 i = 0; i < node->mNumChildren; ++i)
        {
            find_skeleton_roots_recursive(node->mChildren[i], boneNames, roots);
        }
    }

    static std::vector<aiNode*> find_skeleton_roots(const aiScene* scene)
    {
        std::vector<aiNode*> roots;

        std::unordered_set<std::string> boneNames;

        for (uint32 meshIndex = 0; meshIndex < scene->mNumMeshes; ++meshIndex)
        {
            aiMesh* mesh = scene->mMeshes[meshIndex];

            for (uint32 boneIndex = 0; boneIndex < mesh->mNumBones; ++boneIndex)
            {
                aiBone* bone = mesh->mBones[boneIndex];
                std::string boneName = clean_bone_name(bone->mName.C_Str());
                boneNames.insert(boneName);
            }
        }

        find_skeleton_roots_recursive(scene->mRootNode, boneNames, roots);

        return roots;
    }

    static void collect_bone_names_recursive(aiNode* node, std::unordered_set<std::string>& boneNames)
    {
        std::string nodeName = clean_bone_name(node->mName.C_Str());
        boneNames.insert(nodeName);

        for (uint32 i = 0; i < node->mNumChildren; ++i)
        {
            collect_bone_names_recursive(node->mChildren[i], boneNames);
        }
    }

    static void collect_skeleton_bones(const aiScene* scene,
        aiNode* skeletonRoot,
        std::unordered_map<std::string, aiBone*>& skeletonBones)
    {
        std::unordered_set<std::string> boneNames;
        collect_bone_names_recursive(skeletonRoot, boneNames);

        for (uint32 meshIndex = 0; meshIndex < scene->mNumMeshes; ++meshIndex)
        {
            aiMesh* mesh = scene->mMeshes[meshIndex];

            for (uint32 boneIndex = 0; boneIndex < mesh->mNumBones; ++boneIndex)
            {
                aiBone* bone = mesh->mBones[boneIndex];
                std::string boneName = clean_bone_name(bone->mName.C_Str());

                if (boneNames.find(boneName) != boneNames.end())
                {
                    if (skeletonBones.find(boneName) == skeletonBones.end())
                    {
                        skeletonBones[boneName] = bone;
                    }
                }
            }
        }
    }

    static void convert_assimp_matrix(const aiMatrix4x4& aiMat, mat4& outMat)
    {
        outMat.m[0] = aiMat.a1;
        outMat.m[1] = aiMat.b1;
        outMat.m[2] = aiMat.c1;
        outMat.m[3] = aiMat.d1;

        outMat.m[4] = aiMat.a2;
        outMat.m[5] = aiMat.b2;
        outMat.m[6] = aiMat.c2;
        outMat.m[7] = aiMat.d2;

        outMat.m[8] = aiMat.a3;
        outMat.m[9] = aiMat.b3;
        outMat.m[10] = aiMat.c3;
        outMat.m[11] = aiMat.d3;

        outMat.m[12] = aiMat.a4;
        outMat.m[13] = aiMat.b4;
        outMat.m[14] = aiMat.c4;
        outMat.m[15] = aiMat.d4;
    }

    static void process_skeleton_node(aiNode* node,
        aiNode* parentNode,
        const std::unordered_map<std::string, aiBone*>& skeletonBones,
        float scaleFactor,
        animation::SkeletonAssetImportData& result)
    {
        using namespace animation;

        std::string nodeName = clean_bone_name(node->mName.C_Str());

        auto boneIt = skeletonBones.find(nodeName);

        if (boneIt != skeletonBones.end())
        {
            if (result.name_to_joint_id.find(nodeName) != result.name_to_joint_id.end())
            {
                for (uint32 i = 0; i < node->mNumChildren; ++i)
                {
                    process_skeleton_node(node->mChildren[i], node, skeletonBones, scaleFactor, result);
                }
                return;
            }

            aiBone* bone = boneIt->second;

            SkeletonJoint joint;
            joint.name = nodeName;

            aiMatrix4x4 offsetMatrix = bone->mOffsetMatrix;
            convert_assimp_matrix(offsetMatrix, joint.inv_bind_transform);

            trs temp = mat4_to_trs(invert(joint.inv_bind_transform));
            temp.position *= scaleFactor;

            joint.bind_transform = trs_to_mat4(temp);
            joint.inv_bind_transform = invert(joint.bind_transform);

            if (parentNode)
            {
                aiNode* currentParent = parentNode;
                while (currentParent)
                {
                    std::string currentParentName = clean_bone_name(currentParent->mName.C_Str());
                    auto currentParentIt = result.name_to_joint_id.find(currentParentName);

                    if (currentParentIt != result.name_to_joint_id.end())
                    {
                        joint.parent_id = currentParentIt->second;
                        break;
                    }

                    currentParent = currentParent->mParent;
                }
            }

            uint32 jointID = (uint32)result.joints.size();
            result.joints.push_back(joint);
            result.name_to_joint_id[nodeName] = jointID;
        }

        for (uint32 i = 0; i < node->mNumChildren; ++i)
        {
            process_skeleton_node(node->mChildren[i], node, skeletonBones, scaleFactor, result);
        }
    }

    static std::vector<animation::SkeletonAssetImportData> load_skeletons(const aiScene* scene, float scaleFactor, ModelAsset& result_mesh)
    {
        using namespace era_engine::animation;

        std::vector<SkeletonAssetImportData> result;

        std::vector<aiNode*> skeletonRoots = find_skeleton_roots(scene);

        printf("Found %zu skeletons\n", skeletonRoots.size());

        for (size_t i = 0; i < skeletonRoots.size(); ++i)
        {
            SkeletonAssetImportData skeletonData;

            std::unordered_map<std::string, aiBone*> skeletonBones;
            collect_skeleton_bones(scene, skeletonRoots[i], skeletonBones);

            printf("Skeleton %zu: %zu bones\n", i, skeletonBones.size());

            process_skeleton_node(skeletonRoots[i], nullptr, skeletonBones, scaleFactor, skeletonData);

            printf("Processed skeleton %zu: %zu joints\n", i, skeletonData.joints.size());

            for (size_t j = 0; j < skeletonData.joints.size(); ++j)
            {
                printf("  Joint %zu: %s (parent: %d)\n",
                    j,
                    skeletonData.joints[j].name.c_str(),
                    skeletonData.joints[j].parent_id);
            }

            result.push_back(std::move(skeletonData));
        }

        if (result.empty())
        {
            SkeletonAssetImportData singleSkeleton;
            std::unordered_map<std::string, aiBone*> allBones;

            for (uint32 meshIndex = 0; meshIndex < scene->mNumMeshes; ++meshIndex)
            {
                aiMesh* mesh = scene->mMeshes[meshIndex];

                for (uint32 boneIndex = 0; boneIndex < mesh->mNumBones; ++boneIndex)
                {
                    aiBone* bone = mesh->mBones[boneIndex];
                    std::string boneName = clean_bone_name(bone->mName.C_Str());

                    if (allBones.find(boneName) == allBones.end())
                    {
                        allBones[boneName] = bone;
                    }
                }
            }

            if (!allBones.empty())
            {
                printf("No skeleton roots found, using all %zu bones\n", allBones.size());
                process_skeleton_node(scene->mRootNode, nullptr, allBones, scaleFactor, singleSkeleton);

                printf("Processed skeleton: %zu joints\n", singleSkeleton.joints.size());

                for (size_t j = 0; j < singleSkeleton.joints.size(); ++j)
                {
                    printf("  Joint %zu: %s (parent: %d)\n",
                        j,
                        singleSkeleton.joints[j].name.c_str(),
                        singleSkeleton.joints[j].parent_id);
                }

                result.push_back(std::move(singleSkeleton));
            }
        }

        return result;
    }

    static std::vector<aiNodeAnim*> find_all_node_anims_for_bone(
        const aiAnimation* animation,
        const std::string& bone_name)
    {
        std::vector<aiNodeAnim*> result;
        std::string clean_bone = clean_bone_name(bone_name);

        for (uint32 i = 0; i < animation->mNumChannels; ++i)
        {
            aiNodeAnim* nodeAnim = animation->mChannels[i];
            std::string animName = nodeAnim->mNodeName.C_Str();
            std::string cleanAnimName = clean_bone_name(animName);

            if (cleanAnimName == clean_bone || animName == bone_name)
            {
                result.push_back(nodeAnim);
            }
        }

        return result;
    }

    static trs sample_node_animation_component(
        const aiNodeAnim* nodeAnim,
        float time,
        float scaleFactor,
        float ticksPerSecond)
    {
        trs result = trs::identity;

        if (!nodeAnim)
            return result;

        if (nodeAnim->mNumPositionKeys > 0)
        {
            aiVector3D position;

            float firstTime = (float)nodeAnim->mPositionKeys[0].mTime / ticksPerSecond;
            float lastTime = (float)nodeAnim->mPositionKeys[nodeAnim->mNumPositionKeys - 1].mTime / ticksPerSecond;

            if (time <= firstTime)
            {
                position = nodeAnim->mPositionKeys[0].mValue;
            }
            else if (time >= lastTime)
            {
                position = nodeAnim->mPositionKeys[nodeAnim->mNumPositionKeys - 1].mValue;
            }
            else
            {
                for (uint32 i = 0; i < nodeAnim->mNumPositionKeys - 1; ++i)
                {
                    float t0 = (float)nodeAnim->mPositionKeys[i].mTime / ticksPerSecond;
                    float t1 = (float)nodeAnim->mPositionKeys[i + 1].mTime / ticksPerSecond;

                    if (time >= t0 && time <= t1)
                    {
                        float alpha = (t1 > t0) ? (time - t0) / (t1 - t0) : 0.0f;

                        aiVector3D p0 = nodeAnim->mPositionKeys[i].mValue;
                        aiVector3D p1 = nodeAnim->mPositionKeys[i + 1].mValue;

                        position = p0 + (p1 - p0) * alpha;
                        break;
                    }
                }
            }

            result.position = vec3(position.x, position.y, position.z) * scaleFactor;
        }

        if (nodeAnim->mNumRotationKeys > 0)
        {
            aiQuaternion rotation;

            float firstTime = (float)nodeAnim->mRotationKeys[0].mTime / ticksPerSecond;
            float lastTime = (float)nodeAnim->mRotationKeys[nodeAnim->mNumRotationKeys - 1].mTime / ticksPerSecond;

            if (time <= firstTime)
            {
                rotation = nodeAnim->mRotationKeys[0].mValue;
            }
            else if (time >= lastTime)
            {
                rotation = nodeAnim->mRotationKeys[nodeAnim->mNumRotationKeys - 1].mValue;
            }
            else
            {
                for (uint32 i = 0; i < nodeAnim->mNumRotationKeys - 1; ++i)
                {
                    float t0 = (float)nodeAnim->mRotationKeys[i].mTime / ticksPerSecond;
                    float t1 = (float)nodeAnim->mRotationKeys[i + 1].mTime / ticksPerSecond;

                    if (time >= t0 && time <= t1)
                    {
                        float alpha = (t1 > t0) ? (time - t0) / (t1 - t0) : 0.0f;

                        aiQuaternion q0 = nodeAnim->mRotationKeys[i].mValue;
                        aiQuaternion q1 = nodeAnim->mRotationKeys[i + 1].mValue;

                        aiQuaternion::Interpolate(rotation, q0, q1, alpha);
                        break;
                    }
                }
            }

            result.rotation = quat(rotation.x, rotation.y, rotation.z, rotation.w);
        }

        if (nodeAnim->mNumScalingKeys > 0)
        {
            aiVector3D scaling;

            float firstTime = (float)nodeAnim->mScalingKeys[0].mTime / ticksPerSecond;
            float lastTime = (float)nodeAnim->mScalingKeys[nodeAnim->mNumScalingKeys - 1].mTime / ticksPerSecond;

            if (time <= firstTime)
            {
                scaling = nodeAnim->mScalingKeys[0].mValue;
            }
            else if (time >= lastTime)
            {
                scaling = nodeAnim->mScalingKeys[nodeAnim->mNumScalingKeys - 1].mValue;
            }
            else
            {
                for (uint32 i = 0; i < nodeAnim->mNumScalingKeys - 1; ++i)
                {
                    float t0 = (float)nodeAnim->mScalingKeys[i].mTime / ticksPerSecond;
                    float t1 = (float)nodeAnim->mScalingKeys[i + 1].mTime / ticksPerSecond;

                    if (time >= t0 && time <= t1)
                    {
                        float alpha = (t1 > t0) ? (time - t0) / (t1 - t0) : 0.0f;

                        aiVector3D s0 = nodeAnim->mScalingKeys[i].mValue;
                        aiVector3D s1 = nodeAnim->mScalingKeys[i + 1].mValue;

                        scaling = s0 + (s1 - s0) * alpha;
                        break;
                    }
                }
            }

            result.scale = vec3(scaling.x, scaling.y, scaling.z);
        }

        return result;
    }

    static trs get_local_bind_pose(
        const ref<animation::Skeleton>& skeleton,
        uint32 joint_id)
    {
        const auto& joint = skeleton->joints[joint_id];

        trs global_bind = mat4_to_trs(joint.bind_transform);

        if (joint.parent_id == INVALID_JOINT)
        {
            return global_bind;
        }

        const auto& parent_joint = skeleton->joints[joint.parent_id];
        trs parent_global_bind = mat4_to_trs(parent_joint.bind_transform);

        trs parent_inverse = invert(parent_global_bind);

        trs local_bind = parent_inverse * global_bind;

        return local_bind;
    }

    static std::vector<animation::AnimationClipAssetImportData> load_animations(
        const aiScene* scene,
        const ref<animation::Skeleton>& skeleton,
        float scaleFactor)
    {
        using namespace animation;

        std::vector<AnimationClipAssetImportData> result;

        if (!scene->mNumAnimations)
            return result;

        for (uint32 animIndex = 0; animIndex < scene->mNumAnimations; ++animIndex)
        {
            aiAnimation* animation = scene->mAnimations[animIndex];

            AnimationClipAssetImportData animData;
            animData.name = animation->mName.C_Str();
            animData.duration = (float)animation->mDuration / (float)animation->mTicksPerSecond;

            printf("Processing animation: %s (duration: %f seconds, tps: %f)\n",
                animData.name.c_str(), animData.duration, animation->mTicksPerSecond);

            const float sampleRate = 30.0f;
            uint32 numSamples = (uint32)std::ceil(animData.duration * sampleRate) + 1;

            for (uint32 sample = 0; sample < numSamples; ++sample)
            {
                float time = (float)sample / sampleRate;
                if (time > animData.duration)
                    time = animData.duration;
                animData.timestamps.push_back(time);
            }

            for (uint32 sample = 0; sample < numSamples; ++sample)
            {
                float time = animData.timestamps[sample];
                SkeletonPose pose((uint32)skeleton->joints.size());

                for (uint32 jointId = 0; jointId < skeleton->joints.size(); ++jointId)
                {
                    const auto& joint = skeleton->joints[jointId];

                    std::vector<aiNodeAnim*> nodeAnims = find_all_node_anims_for_bone(animation, joint.name);

                    trs final_transform;

                    if (!nodeAnims.empty())
                    {
                        trs local_bind_pose = get_local_bind_pose(skeleton, jointId);

                        final_transform = local_bind_pose;

                        bool has_position_anim = false;
                        bool has_rotation_anim = false;
                        bool has_scale_anim = false;

                        for (aiNodeAnim* nodeAnim : nodeAnims)
                        {
                            trs component_transform = sample_node_animation_component(
                                nodeAnim,
                                time,
                                scaleFactor,
                                (float)animation->mTicksPerSecond
                            );

                            if (nodeAnim->mNumPositionKeys > 1)
                            {
                                final_transform.position = component_transform.position;
                                has_position_anim = true;
                            }
                            if (nodeAnim->mNumRotationKeys > 1)
                            {
                                final_transform.rotation = component_transform.rotation;
                                has_rotation_anim = true;
                            }
                            if (nodeAnim->mNumScalingKeys > 1)
                            {
                                final_transform.scale = component_transform.scale;
                                has_scale_anim = true;
                            }
                        }

                        if (!has_position_anim)
                        {
                            final_transform.position = local_bind_pose.position;
                        }
                        if (!has_rotation_anim)
                        {
                            final_transform.rotation = local_bind_pose.rotation;
                        }
                        if (!has_scale_anim)
                        {
                            final_transform.scale = local_bind_pose.scale;
                        }
                    }
                    else
                    {
                        final_transform = get_local_bind_pose(skeleton, jointId);
                    }

                    JointTransform jointTransform;
                    jointTransform.set_transform(final_transform);
                    pose.set_joint_transform(jointTransform, jointId);
                }

                animData.poses.push_back(pose);
            }

            printf("Animation '%s': %u samples processed\n",
                animData.name.c_str(), numSamples);

            result.push_back(std::move(animData));
        }

        printf("Loaded %zu animations\n", result.size());
        return result;
    }

	ERA_CORE_API ModelAsset universal_load_model(const fs::path& path, uint32 flags)
	{
		ModelAsset result;
        result.flags = flags;

		Assimp::Importer importer;

        importer.SetPropertyBool(AI_CONFIG_IMPORT_FBX_PRESERVE_PIVOTS, false);

		const aiScene* scene = importer.ReadFile(path.string(),
            aiProcess_Triangulate |
            aiProcess_GenNormals |
            aiProcess_FixInfacingNormals |
            aiProcess_CalcTangentSpace |
            aiProcess_JoinIdenticalVertices |
            aiProcess_ConvertToLeftHanded |
            aiProcess_ImproveCacheLocality |
            aiProcess_GenUVCoords |
            aiProcess_SortByPType);

		if (scene == nullptr || 
			scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || 
			scene->mRootNode == nullptr)
		{
			LOG_ERROR(importer.GetErrorString());
			return result;
		}

        load_materials(scene, result);

        float scaleFactor = 1.0f;
        if (flags & mesh_creation_flags_sm_to_m)
        {
            scaleFactor = 0.01f;
        }
        else if (flags & mesh_creation_flags_m_to_sm)
        {
            scaleFactor = 100.0f;
        }

        load_meshes(scene, scaleFactor, result);

        std::vector<animation::SkeletonAssetImportData> skeletons = load_skeletons(scene, scaleFactor, result);
        std::vector<ref<animation::Skeleton>> imported_skeletons = animation::SkeletonImportUtils::import_skeletons(skeletons, path, flags);

        if (!imported_skeletons.empty())
        {
            std::vector<animation::AnimationClipAssetImportData> animations = load_animations(scene, imported_skeletons[0], scaleFactor);
            animation::AnimationAssetClipUtils::import_animations(animations, imported_skeletons[0], path, flags);
        }

		return result;
	}
}