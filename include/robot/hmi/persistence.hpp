#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "robot/hmi/parameter_transaction.hpp"

namespace robot {

constexpr std::uint32_t kParameterProfileMagic = 0x4C465037u;  // "7PFL"
constexpr std::uint16_t kParameterProfileSchema = 1;

inline std::uint32_t crc32(const std::uint8_t* data, std::size_t size) noexcept {
  std::uint32_t crc = 0xFFFFFFFFu;
  for (std::size_t i = 0; i < size; ++i) {
    crc ^= data[i];
    for (std::uint8_t bit = 0; bit < 8; ++bit)
      crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
  }
  return ~crc;
}

template <std::size_t MaxEntries>
struct ParameterProfile {
  std::uint32_t generation{};
  std::uint32_t robot_id_hash{};
  std::uint32_t registry_revision{};
  ProfileState state{ProfileState::Draft};
  std::array<ParameterValue, MaxEntries> values{};
  std::size_t count{};
};

template <std::size_t MaxEntries>
constexpr std::size_t parameterProfileMaxBytes() noexcept {
  return 28u + MaxEntries * 12u;
}

namespace detail {

template <std::size_t Capacity>
inline bool putU16(std::array<std::uint8_t, Capacity>& out, std::size_t& cursor,
                   std::uint16_t value) noexcept {
  if (cursor + 2 > Capacity) return false;
  out[cursor++] = static_cast<std::uint8_t>(value);
  out[cursor++] = static_cast<std::uint8_t>(value >> 8);
  return true;
}

template <std::size_t Capacity>
inline bool putU32(std::array<std::uint8_t, Capacity>& out, std::size_t& cursor,
                   std::uint32_t value) noexcept {
  if (cursor + 4 > Capacity) return false;
  for (std::uint8_t shift = 0; shift < 32; shift += 8)
    out[cursor++] = static_cast<std::uint8_t>(value >> shift);
  return true;
}

template <std::size_t Capacity>
inline bool putU64(std::array<std::uint8_t, Capacity>& out, std::size_t& cursor,
                   std::uint64_t value) noexcept {
  if (cursor + 8 > Capacity) return false;
  for (std::uint8_t shift = 0; shift < 64; shift += 8)
    out[cursor++] = static_cast<std::uint8_t>(value >> shift);
  return true;
}

inline bool getU16(const std::uint8_t* data, std::size_t size,
                   std::size_t& cursor, std::uint16_t& value) noexcept {
  if (cursor + 2 > size) return false;
  value = static_cast<std::uint16_t>(data[cursor]) |
          static_cast<std::uint16_t>(data[cursor + 1]) << 8;
  cursor += 2;
  return true;
}

inline bool getU32(const std::uint8_t* data, std::size_t size,
                   std::size_t& cursor, std::uint32_t& value) noexcept {
  if (cursor + 4 > size) return false;
  value = 0;
  for (std::uint8_t shift = 0; shift < 32; shift += 8)
    value |= static_cast<std::uint32_t>(data[cursor++]) << shift;
  return true;
}

inline bool getU64(const std::uint8_t* data, std::size_t size,
                   std::size_t& cursor, std::uint64_t& value) noexcept {
  if (cursor + 8 > size) return false;
  value = 0;
  for (std::uint8_t shift = 0; shift < 64; shift += 8)
    value |= static_cast<std::uint64_t>(data[cursor++]) << shift;
  return true;
}

}  // namespace detail

template <std::size_t MaxEntries>
struct EncodedParameterProfile {
  std::array<std::uint8_t, parameterProfileMaxBytes<MaxEntries>()> bytes{};
  std::size_t size{};
};

template <std::size_t MaxEntries>
inline bool encodeParameterProfile(
    const ParameterProfile<MaxEntries>& profile,
    EncodedParameterProfile<MaxEntries>& encoded) noexcept {
  if (profile.count > MaxEntries || profile.generation == 0 ||
      profile.robot_id_hash == 0 || profile.registry_revision == 0)
    return false;
  encoded = {};
  std::size_t cursor{};
  using namespace detail;
  if (!putU32(encoded.bytes, cursor, kParameterProfileMagic) ||
      !putU16(encoded.bytes, cursor, kParameterProfileSchema) ||
      !putU16(encoded.bytes, cursor,
              static_cast<std::uint16_t>(profile.state)) ||
      !putU32(encoded.bytes, cursor, profile.generation) ||
      !putU32(encoded.bytes, cursor, profile.robot_id_hash) ||
      !putU32(encoded.bytes, cursor, profile.registry_revision) ||
      !putU32(encoded.bytes, cursor, static_cast<std::uint32_t>(profile.count)))
    return false;
  for (std::size_t i = 0; i < profile.count; ++i) {
    std::uint64_t bits{};
    static_assert(sizeof(bits) == sizeof(profile.values[i].value),
                  "persistence requires IEEE-sized double");
    std::memcpy(&bits, &profile.values[i].value, sizeof(bits));
    if (!putU32(encoded.bytes, cursor, profile.values[i].id) ||
        !putU64(encoded.bytes, cursor, bits))
      return false;
  }
  const std::uint32_t checksum = crc32(encoded.bytes.data(), cursor);
  if (!putU32(encoded.bytes, cursor, checksum)) return false;
  encoded.size = cursor;
  return true;
}

template <std::size_t RegistryCapacity, std::size_t MaxEntries>
inline bool decodeParameterProfile(
    const std::uint8_t* data, std::size_t size,
    const ParameterRegistry<RegistryCapacity>& registry,
    const ParameterBackend& backend, std::uint32_t expected_robot_id_hash,
    ParameterProfile<MaxEntries>& profile) noexcept {
  constexpr std::size_t kHeaderAndCrcBytes = 28;
  if (data == nullptr || size < kHeaderAndCrcBytes ||
      (size - kHeaderAndCrcBytes) % 12 != 0)
    return false;
  std::uint32_t stored_crc{};
  std::size_t crc_cursor = size - 4;
  if (!detail::getU32(data, size, crc_cursor, stored_crc) ||
      stored_crc != crc32(data, size - 4))
    return false;

  ParameterProfile<MaxEntries> decoded{};
  std::size_t cursor{};
  std::uint32_t magic{}, count{};
  std::uint16_t schema{}, state{};
  if (!detail::getU32(data, size, cursor, magic) ||
      !detail::getU16(data, size, cursor, schema) ||
      !detail::getU16(data, size, cursor, state) ||
      !detail::getU32(data, size, cursor, decoded.generation) ||
      !detail::getU32(data, size, cursor, decoded.robot_id_hash) ||
      !detail::getU32(data, size, cursor, decoded.registry_revision) ||
      !detail::getU32(data, size, cursor, count))
    return false;
  if (magic != kParameterProfileMagic || schema != kParameterProfileSchema ||
      decoded.generation == 0 ||
      decoded.robot_id_hash != expected_robot_id_hash ||
      decoded.registry_revision != registry.revision() || count > MaxEntries ||
      state > static_cast<std::uint16_t>(ProfileState::Retired) ||
      size != kHeaderAndCrcBytes + static_cast<std::size_t>(count) * 12)
    return false;
  decoded.state = static_cast<ProfileState>(state);
  decoded.count = count;
  for (std::size_t i = 0; i < decoded.count; ++i) {
    std::uint32_t encoded_id{};
    std::uint64_t bits{};
    if (!detail::getU32(data, size, cursor, encoded_id) ||
        !detail::getU64(data, size, cursor, bits))
      return false;
    if (encoded_id > 0xFFFFu) return false;
    decoded.values[i].id = static_cast<ParameterId>(encoded_id);
    std::memcpy(&decoded.values[i].value, &bits, sizeof(bits));
    const auto* descriptor = registry.find(decoded.values[i].id);
    if (descriptor == nullptr || !descriptor->available ||
        descriptor->persistence == PersistencePolicy::SessionOnly ||
        decoded.values[i].value < descriptor->min_value ||
        decoded.values[i].value > descriptor->max_value ||
        !backend.validate(decoded.values[i].id, decoded.values[i].value))
      return false;
    for (std::size_t j = 0; j < i; ++j)
      if (decoded.values[j].id == decoded.values[i].id) return false;
  }
  profile = decoded;
  return true;
}

class ParameterProfileSlot {
 public:
  virtual ~ParameterProfileSlot() = default;
  virtual bool read(std::uint8_t* destination, std::size_t capacity,
                    std::size_t& size) const noexcept = 0;
  virtual bool write(const std::uint8_t* source, std::size_t size) noexcept = 0;
};

template <std::size_t RegistryCapacity, std::size_t MaxEntries>
class DualSlotParameterStore {
 public:
  DualSlotParameterStore(const ParameterRegistry<RegistryCapacity>& registry,
                         const ParameterBackend& backend,
                         ParameterProfileSlot& slot_a,
                         ParameterProfileSlot& slot_b,
                         std::uint32_t robot_id_hash) noexcept
      : registry_(registry),
        backend_(backend),
        slots_{&slot_a, &slot_b},
        robot_id_hash_(robot_id_hash) {}

  bool load(ParameterProfile<MaxEntries>& profile) const noexcept {
    ParameterProfile<MaxEntries> candidates[2]{};
    bool valid[2]{};
    for (std::size_t i = 0; i < 2; ++i) valid[i] = read(i, candidates[i]);
    if (!valid[0] && !valid[1]) return false;
    const std::size_t selected =
        valid[1] && (!valid[0] || candidates[1].generation >
                                      candidates[0].generation)
            ? 1
            : 0;
    profile = candidates[selected];
    return true;
  }

  bool save(ParameterProfile<MaxEntries> profile) noexcept {
    ParameterProfile<MaxEntries> existing[2]{};
    const bool valid_a = read(0, existing[0]);
    const bool valid_b = read(1, existing[1]);
    std::uint32_t latest{};
    if (valid_a) latest = existing[0].generation;
    if (valid_b && existing[1].generation > latest)
      latest = existing[1].generation;
    const std::size_t target =
        !valid_a ? 0 : (!valid_b ? 1 : (existing[0].generation <=
                                        existing[1].generation
                                            ? 0
                                            : 1));
    profile.generation = latest + 1;
    if (profile.generation == 0) return false;
    profile.robot_id_hash = robot_id_hash_;
    profile.registry_revision = registry_.revision();
    EncodedParameterProfile<MaxEntries> encoded{};
    if (!encodeParameterProfile(profile, encoded) ||
        !slots_[target]->write(encoded.bytes.data(), encoded.size))
      return false;
    ParameterProfile<MaxEntries> readback{};
    if (!read(target, readback) ||
        readback.generation != profile.generation ||
        readback.robot_id_hash != profile.robot_id_hash ||
        readback.registry_revision != profile.registry_revision ||
        readback.state != profile.state || readback.count != profile.count)
      return false;
    for (std::size_t i = 0; i < profile.count; ++i)
      if (readback.values[i].id != profile.values[i].id ||
          readback.values[i].value != profile.values[i].value)
        return false;
    return true;
  }

 private:
  bool read(std::size_t index,
            ParameterProfile<MaxEntries>& profile) const noexcept {
    EncodedParameterProfile<MaxEntries> encoded{};
    if (!slots_[index]->read(encoded.bytes.data(), encoded.bytes.size(),
                             encoded.size))
      return false;
    return decodeParameterProfile(encoded.bytes.data(), encoded.size, registry_,
                                  backend_, robot_id_hash_, profile);
  }

  const ParameterRegistry<RegistryCapacity>& registry_;
  const ParameterBackend& backend_;
  std::array<ParameterProfileSlot*, 2> slots_{};
  std::uint32_t robot_id_hash_{};
};

}  // namespace robot
