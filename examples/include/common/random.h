//
// Created by waxz on 5/22/23.
//

#ifndef CMAKE_SUPER_BUILD_RANDOM_H
#define CMAKE_SUPER_BUILD_RANDOM_H

#include <random>
#include <algorithm>
#include <memory>
#include <cassert>

namespace common{
    inline std::mt19937& rng(std::seed_seq seed_sequence = {})  {
        thread_local auto first = true;
        thread_local auto generator = [&seed_sequence]() {
            first = false;
            if (seed_sequence.size() > 0) return std::mt19937(seed_sequence);
            auto seed_data = std::array<int, std::mt19937::state_size>();
            std::random_device random_device;
            std::generate_n(seed_data.data(), seed_data.size(), std::ref(random_device));
            std::seed_seq sequence (std::begin(seed_data), std::end(seed_data));
            return std::mt19937(sequence);
        }();

        if (!first && seed_sequence.size() > 0)
            throw std::runtime_error("rng cannot be re-seeded on this thread");
        return generator;
    }


    /**
 * @brief Get a uniform real number in a given range
 *
 * @param lower Lower bound, inclusive
 * @param upper Upper bound, exclusive
 *
 * @tparam RealType Floating point type
 *
 * @return Uniform real in range [lower, upper)
 */
    template <typename RealType>
    auto uniform_real(RealType lower, RealType upper) {
        static_assert(std::is_floating_point<RealType>::value , "RealType is not floating_point");
        assert(lower < upper);
        return std::uniform_real_distribution<>(lower, upper)(rng());
    }


    template <typename RealType>
    auto normal_real(RealType mean, RealType stddev) {
        static_assert(std::is_floating_point<RealType>::value , "RealType is not floating_point");
        return std::normal_distribution<>(mean, stddev)(rng());
    }

/**
 * @brief Get a uniform integer number in a given range
 *
 * @param lower Lower bound, inclusive
 * @param upper Upper bound, inclusive
 *
 * @tparam IntType Integral type
 *
 * @return Uniform integer in range [lower, upper]
 */
    template <typename IntType>
    auto uniform_int(IntType lower, IntType upper) {
        static_assert(std::is_integral<IntType>::value , "RealType is not integral");
        assert(lower <= upper);
        return std::uniform_int_distribution<>(lower, upper)(rng());
    }
}
#endif //CMAKE_SUPER_BUILD_RANDOM_H
