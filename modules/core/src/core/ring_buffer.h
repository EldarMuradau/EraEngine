#pragma once
#include <optional>
#include <vector>
#include <mutex>
#include <shared_mutex>
#include <cassert>

namespace era_engine
{
    template<typename T>
    class RingBuffer {
    public:
        explicit RingBuffer(size_t _capacity)
            : capacity(_capacity)
            , buffer(_capacity)
        {
        }

        RingBuffer(const RingBuffer&) = delete;
        RingBuffer& operator=(const RingBuffer&) = delete;

        RingBuffer(RingBuffer&& other) noexcept
            : capacity(other.capacity)
            , buffer(std::move(other.buffer))
            , head(other.head)
            , tail(other.tail)
            , full(other.full)
        {
            other.capacity = 0;
            other.head = 0;
            other.tail = 0;
            other.full = false;
        }

        void push(const T& value)
        {
            std::unique_lock lock(mutex);

            buffer[tail] = value;

            if (full)
            {
                head = (head + 1) % capacity;
            }

            tail = (tail + 1) % capacity;
            full = (tail == head);
        }

        void push(T&& value)
        {
            std::unique_lock lock(mutex);

            buffer[tail] = std::move(value);

            if (full)
            {
                head = (head + 1) % capacity;
            }

            tail = (tail + 1) % capacity;
            full = (tail == head);
        }

        T& operator[](int index)
        {
            std::shared_lock lock(mutex);

            if (index >= get_size())
            {
                throw std::out_of_range("RingBuffer index out of range");
            }

            size_t real_index = (head + index) % capacity;
            return buffer[real_index];
        }

        const T& operator[](int index) const
        {
            std::shared_lock lock(mutex);

            if (index >= get_size())
            {
                throw std::out_of_range("RingBuffer index out of range");
            }

            size_t real_index = (head + index) % capacity;
            return buffer[real_index];
        }

        std::optional<T> pop()
        {
            std::unique_lock lock(mutex);

            if (is_empty())
            {
                return std::nullopt;
            }

            T value = std::move(buffer[head]);
            head = (head + 1) % capacity;
            full = false;

            return value;
        }

        std::optional<T> front() const
        {
            std::shared_lock lock(mutex);

            if (is_empty())
            {
                return std::nullopt;
            }

            return buffer[head];
        }

        std::optional<T> get_back() const
        {
            std::shared_lock lock(mutex);

            if (is_empty())
            {
                return std::nullopt;
            }

            size_t last_pos = (tail == 0) ? capacity - 1 : tail - 1;
            return buffer[last_pos];
        }

        size_t get_size() const
        {
            std::shared_lock lock(mutex);

            if (is_empty())
            {
                return 0;
            }

            if (full)
            {
                return capacity;
            }

            if (tail > head)
            {
                return tail - head;
            }
            else
            {
                return capacity - head + tail;
            }
        }

        size_t get_capacity() const
        {
            return capacity;
        }

        bool is_empty() const
        {
            return (!full && (head == tail));
        }

        bool is_full() const
        {
            return full;
        }

        void clear()
        {
            std::unique_lock lock(mutex);
            head = 0;
            tail = 0;
            full = false;
        }

    private:
        mutable std::shared_mutex mutex;
        std::vector<T> buffer;

        size_t capacity = 0;
        size_t head = 0;
        size_t tail = 0;

        bool full = false;
    };
}