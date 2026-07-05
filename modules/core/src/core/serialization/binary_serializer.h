#pragma once

#include "core_api.h"

#include "core/math.h"
#include "core/io/binary_data.h"
#include "core/io/binary_data_archive.h"

#include <algorithm>
#include <cstring>
#include <memory>
#include <numeric>
#include <utility>

namespace era_engine
{
    struct BinarySerializer
    {
        template <class T>
        static inline BinaryDataArchive serialize(const T& obj);

        template <class T>
        static inline size_t serialize(uint8* out, const T& obj);

        template <class Head, class... Tail>
        static inline size_t serialize(uint8* out, const Head& hd, Tail&... tl);

        template <class T>
        static inline size_t deserialize(const uint8* in, size_t buff_size, T& obj);

        template <class Head, class... Tail>
        static inline size_t deserialize(const uint8* in, size_t buff_size, Head& hd, Tail&... tl);

        template <class Head, class... Tail>
        static inline size_t deserialize(const BinaryData& binary, Head& hd, Tail&... tl);

        template <class T>
        static inline size_t get_size(const T& obj);

        template <class Head, class... Tail>
        static inline size_t get_size(const Head& hd, Tail&... tl);
    };
}

#define ERA_BINARY_SERIALIZE(...)                                                     \
    inline size_t serialize(uint8* out) const                                         \
    {                                                                                 \
        return era_engine::BinarySerializer::serialize(out, __VA_ARGS__);             \
    }                                                                                 \
    inline size_t deserialize(const uint8* in, size_t buff_size)                      \
    {                                                                                 \
        return era_engine::BinarySerializer::deserialize(in, buff_size, __VA_ARGS__); \
    }                                                                                 \
    inline size_t get_size() const                                                    \
    {                                                                                 \
        return era_engine::BinarySerializer::get_size(__VA_ARGS__);                   \
    }

#define ERA_BASE_VIRTUAL_BINARY_SERIALIZE(...)                                        \
    virtual size_t serialize(uint8* out) const                                        \
    {                                                                                 \
        return era_engine::BinarySerializer::serialize(out, __VA_ARGS__);             \
    }                                                                                 \
    virtual size_t deserialize(const uint8* in, size_t buff_size)                     \
    {                                                                                 \
        return era_engine::BinarySerializer::deserialize(in, buff_size, __VA_ARGS__); \
    }                                                                                 \
    virtual size_t get_size() const                                                   \
    {                                                                                 \
        return era_engine::BinarySerializer::get_size(__VA_ARGS__);                   \
    }

#define ERA_VIRTUAL_BINARY_SERIALIZE(...)                                             \
    size_t serialize(uint8* out) const override                                       \
    {                                                                                 \
        return era_engine::BinarySerializer::serialize(out, __VA_ARGS__);             \
    }                                                                                 \
    size_t deserialize(const uint8* in, size_t buff_size) override                    \
    {                                                                                 \
        return era_engine::BinarySerializer::deserialize(in, buff_size, __VA_ARGS__); \
    }                                                                                 \
    size_t get_size() const override                                                  \
    {                                                                                 \
        return era_engine::BinarySerializer::get_size(__VA_ARGS__);                   \
    }

#include "core/serialization/private/binary_serializer_impl.h"