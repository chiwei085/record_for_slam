#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace rtabmap_semantic_mapping
{

inline std::array<std::uint8_t, 3> class_color_rgb(const int class_id) {
    switch (class_id) {
        case 73:
            return {70U, 130U, 180U};  // book
        case 56:
            return {60U, 179U, 113U};  // chair
        case 62:
            return {220U, 20U, 60U};  // tv
        case 63:
            return {255U, 140U, 0U};  // laptop
        case 60:
            return {138U, 43U, 226U};  // dining table
        case 0:
            return {47U, 79U, 79U};  // person
        default:
            break;
    }

    const std::size_t hash =
        static_cast<std::size_t>(class_id >= 0 ? class_id : -class_id + 997);
    return {
        static_cast<std::uint8_t>(50U + (hash % 156U)),
        static_cast<std::uint8_t>(50U + ((hash / 157U) % 156U)),
        static_cast<std::uint8_t>(50U + ((hash / 24649U) % 156U)),
    };
}

inline std::array<float, 3> class_color_normalized(const int class_id) {
    const auto rgb = class_color_rgb(class_id);
    return {
        static_cast<float>(rgb[0]) / 255.0F,
        static_cast<float>(rgb[1]) / 255.0F,
        static_cast<float>(rgb[2]) / 255.0F,
    };
}

}  // namespace rtabmap_semantic_mapping
