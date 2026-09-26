#include "mars/env.hpp"
#include "mars/biome_bank.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <vector>

namespace mars {

void Env::select_mechanic_layout(uint64_t seed) {
  rng_.seed(seed ^ 0xd1b54a32d192ed03ULL);
  const auto& bank = biome_registry();
  std::vector<int> eligible_biomes;
  eligible_biomes.reserve(bank.size());
  if (config_.fixed_biome_id >= 0 && config_.fixed_biome_id < static_cast<int>(bank.size())) {
    eligible_biomes.push_back(config_.fixed_biome_id);
  } else {
    for (int i = 0; i < static_cast<int>(bank.size()); ++i) {
      const Biome& biome = *bank[static_cast<size_t>(i)];
      const bool eligible =
          config_.biome_split == 0 ||
          (config_.biome_split == 1 && biome.split() == BiomeSplit::Train) ||
          (config_.biome_split == 3 && biome.is_anchor());
      if (eligible) eligible_biomes.push_back(i);
    }
  }
  if (eligible_biomes.empty()) {

    eligible_biomes.push_back(builtin_biome_id(MechanicType::Normal));
  }
  std::uniform_real_distribution<float> u(0.0f, 1.0f);
  const float safe_min = clamp(std::min(config_.difficulty_safe_fraction_min,
                                        config_.difficulty_safe_fraction_max), 0.0f, 0.40f);
  const float safe_max = clamp(std::max(config_.difficulty_safe_fraction_min,
                                        config_.difficulty_safe_fraction_max), safe_min, 0.45f);
  difficulty_safe_fraction_ = safe_min + (safe_max - safe_min) * u(rng_);
  const FrozenMechanismStack evaluation_stack{
      config_.evaluation_stack_types,
      std::clamp(config_.evaluation_stack_count, 0, kMaxActiveMechanisms),
      BiomeSplit::Builtin};
  const bool force_evaluation_stack = evaluation_stack.count > 0;
  const auto build_layers = [&]() {
    mechanic_layout_.layer_count = 0;

    std::uniform_real_distribution<float> region_length(55.0f, 120.0f);
    float begin = -20.0f;
    const auto id_for_type = [&](MechanicType type) {
      for (int id : eligible_biomes) {
        if (bank[static_cast<size_t>(id)]->visual_type() == type) return id;
      }
      for (int id = 0; id < static_cast<int>(bank.size()); ++id) {
        if (bank[static_cast<size_t>(id)]->is_anchor() &&
            bank[static_cast<size_t>(id)]->visual_type() == type) return id;
      }
      return eligible_biomes[static_cast<size_t>(rng_() % eligible_biomes.size())];
    };
    std::array<int, kFrozenMechanismStacks.size()> allowed{};
    int allowed_count = 0;
    for (int i = 0; i < static_cast<int>(kFrozenMechanismStacks.size()); ++i) {
      const auto split = kFrozenMechanismStacks[static_cast<size_t>(i)].split;
      const bool matches = config_.biome_split == 0 ||
          (config_.biome_split == 1 && split == BiomeSplit::Train) ||
          (config_.biome_split == 3 && split == BiomeSplit::Builtin);
      if (matches) allowed[static_cast<size_t>(allowed_count++)] = i;
    }
    if (allowed_count == 0) {
      for (int i = 0; i < static_cast<int>(kFrozenMechanismStacks.size()); ++i) {
        allowed[static_cast<size_t>(allowed_count++)] = i;
      }
    }
    int region = 0;
    while (begin < terrain_.length() + 250.0f &&
           mechanic_layout_.layer_count < kMaxMechanicZones - kMaxActiveMechanisms) {
      const float end = begin + region_length(rng_);
      const float difficulty = course_difficulty((begin + end) * 0.5f);
      const auto& approved = force_evaluation_stack
          ? evaluation_stack
          : kFrozenMechanismStacks[static_cast<size_t>(
                allowed[static_cast<size_t>(rng_() % static_cast<uint64_t>(allowed_count))])];

      const bool calm_region = !force_evaluation_stack && u(rng_) > difficulty;
      const int stack = calm_region ? 1 : approved.count;
      for (int j = 0; j < stack && mechanic_layout_.layer_count < kMaxMechanicZones; ++j) {
        const MechanicType type = calm_region ? MechanicType::Normal
                                               : approved.types[static_cast<size_t>(j)];
        const int id = id_for_type(type);
        const Biome& b = *bank[static_cast<size_t>(id)];
        auto& layer = mechanic_layout_.layers[static_cast<size_t>(mechanic_layout_.layer_count++)];
        layer.begin_x = begin; layer.end_x = end; layer.type = b.visual_type();
        const uint64_t parameter_seed = rng_();
        layer.biome_id = id; layer.params = b.sample_params(parameter_seed);
        prepare_generation_params(layer.params, layer.type, parameter_seed);
        layer.terrain_seed = rng_(); layer.liquid_level = -1.0e9f;
      }
      begin = end; ++region;
    }
  };

  if (config_.fixed_biome_id >= 0 || !config_.chain_biomes) {

    std::uniform_int_distribution<size_t> biome_dist(0, eligible_biomes.size() - 1);
    const int biome_id = eligible_biomes[biome_dist(rng_)];
    const Biome& biome = *bank[static_cast<size_t>(biome_id)];
    mechanic_layout_.count = 1;
    auto& zone = mechanic_layout_.zones[0];
    zone.begin_x = -1000000.0f;
    zone.end_x = 1000000.0f;
    zone.type = biome.visual_type();
    zone.biome_id = biome_id;
    const uint64_t parameter_seed = rng_();
    zone.params = biome.sample_params(parameter_seed);
    prepare_generation_params(zone.params, zone.type, parameter_seed);
    zone.terrain_seed = rng_();
    zone.terrain_surprise_mode = 0;
    zone.terrain_surprise_strength = 0.0f;
    const float zone_difficulty = course_difficulty(config_.terrain.length * 0.5f);
    if (config_.fixed_biome_id < 0 &&
        u(rng_) < clamp(config_.terrain_surprise_probability, 0.0f, 1.0f) *
                      (0.03f + 0.97f * zone_difficulty)) {
      zone.terrain_surprise_mode = 1 + static_cast<int>(u(rng_) * 4.0f) % 4;
      zone.terrain_surprise_strength =
          std::max(0.0f, config_.terrain_surprise_strength) *
          (0.08f + 0.92f * zone_difficulty) * (0.75f + 0.5f * u(rng_));
    }
    zone.liquid_level = -1.0e9f;
    pending_basin_depth_[0] =
        biome.visual_type() == MechanicType::Liquid
            ? 2.5f + zone_difficulty * 4.5f + u(rng_) * (1.0f + 2.0f * zone_difficulty)
            : -1.0f;
    build_layers();
    return;
  }

  std::vector<int> anchors;
  std::vector<int> rest;
  rest.reserve(eligible_biomes.size());
  for (int id : eligible_biomes) {
    if (!bank[static_cast<size_t>(id)]->is_anchor()) rest.push_back(id);
  }
  for (int i = 0; i < static_cast<int>(bank.size()); ++i) {
    if (bank[static_cast<size_t>(i)]->is_anchor()) anchors.push_back(i);
  }
  std::shuffle(anchors.begin(), anchors.end(), rng_);
  std::shuffle(rest.begin(), rest.end(), rng_);
  const std::vector<int>& pool = !rest.empty() ? rest : (!anchors.empty() ? anchors : eligible_biomes);

  const int zone_count = std::clamp(config_.chain_zone_count, 1, kMaxMechanicZones);
  std::vector<int> order;
  order.reserve(static_cast<size_t>(zone_count));
  size_t anchor_cursor = 0;
  size_t pool_cursor = 0;
  int previous_id = -1;
  for (int slot = 0; slot < zone_count; ++slot) {

    const float slot_progress = (static_cast<float>(slot) + 0.5f) /
                                static_cast<float>(std::max(1, zone_count));
    const float slot_x = slot_progress * config_.terrain.length;
    const float difficulty = course_difficulty(slot_x);
    const bool take_anchor = !anchors.empty() && (slot == 0 || u(rng_) > difficulty);
    int candidate;
    if (take_anchor) {
      if (difficulty < 0.18f) {
        candidate = builtin_biome_id(MechanicType::Normal);
      } else {
        candidate = anchors[anchor_cursor % anchors.size()];
        ++anchor_cursor;
      }
    } else {
      candidate = pool[pool_cursor % pool.size()];
      ++pool_cursor;
      if (candidate == previous_id && pool.size() > 1) {
        pool_cursor += 1;
        candidate = pool[pool_cursor % pool.size()];
      }
    }

    const int liquid_id = builtin_biome_id(MechanicType::Liquid);
    const bool previous_was_liquid =
        previous_id >= 0 && bank[static_cast<size_t>(previous_id)]->visual_type() ==
                                MechanicType::Liquid;
    const float liquid_probability = 0.10f + 0.16f * difficulty;
    if (slot > 0 && difficulty >= 0.10f && !previous_was_liquid &&
        u(rng_) < liquid_probability) {
      candidate = liquid_id;
    }
    order.push_back(candidate);
    previous_id = candidate;
  }

  mechanic_layout_.count = static_cast<int>(order.size());
  float cursor = -8.0f;
  std::uniform_real_distribution<float> length_dist(config_.chain_segment_min_length,
                                                      config_.chain_segment_max_length);
  for (int slot = 0; slot < mechanic_layout_.count; ++slot) {
    const int biome_id = order[static_cast<size_t>(slot)];
    const Biome& biome = *bank[static_cast<size_t>(biome_id)];
    auto& zone = mechanic_layout_.zones[static_cast<size_t>(slot)];
    zone.begin_x = cursor;
    const bool last = slot + 1 == mechanic_layout_.count;

    cursor += last ? 1000000.0f : length_dist(rng_);
    zone.end_x = cursor;
    zone.type = biome.visual_type();
    zone.biome_id = biome_id;
    const uint64_t parameter_seed = rng_();
    zone.params = biome.sample_params(parameter_seed);
    prepare_generation_params(zone.params, zone.type, parameter_seed);
    zone.terrain_seed = rng_();
    zone.terrain_surprise_mode = 0;
    zone.terrain_surprise_strength = 0.0f;
    const float zone_probe = last ? std::min(config_.terrain.length, zone.begin_x + 35.0f)
                                  : (zone.begin_x + zone.end_x) * 0.5f;
    const float zone_difficulty = course_difficulty(zone_probe);
    if (config_.fixed_biome_id < 0 &&
        u(rng_) < clamp(config_.terrain_surprise_probability, 0.0f, 1.0f) *
                      (0.03f + 0.97f * zone_difficulty)) {
      zone.terrain_surprise_mode = 1 + static_cast<int>(u(rng_) * 4.0f) % 4;
      zone.terrain_surprise_strength =
          std::max(0.0f, config_.terrain_surprise_strength) *
          (0.08f + 0.92f * zone_difficulty) * (0.75f + 0.5f * u(rng_));
    }
    zone.liquid_level = -1.0e9f;
    pending_basin_depth_[static_cast<size_t>(slot)] =
        biome.visual_type() == MechanicType::Liquid
            ? 2.5f + zone_difficulty * 4.5f + u(rng_) * (1.0f + 2.0f * zone_difficulty)
            : -1.0f;
  }
  build_layers();
}

void Env::finalize_mechanic_layout() {

  for (int slot = 0; slot < mechanic_layout_.count; ++slot) {
    auto& zone = mechanic_layout_.zones[static_cast<size_t>(slot)];
    const Biome& biome = biome_by_id(zone.biome_id);
    const float finite_end = std::min(terrain_.length(), zone.end_x);
    const float profile_x = clamp((std::max(0.0f, zone.begin_x) + finite_end) * 0.5f,
                                  0.0f, terrain_.length());
    const float zone_difficulty = terrain_.difficulty_at(profile_x);
    const float simple_weight = 1.0f - zone_difficulty;
    constexpr float medium_weight = 0.4f;
    const float complex_weight = zone_difficulty;
    const float profile_draw = biome_random01(zone.terrain_seed, 18) *
                               (simple_weight + medium_weight + complex_weight);
    int profile = 1;
    if (profile_draw < simple_weight) {
      profile = biome_random01(zone.terrain_seed, 19) < 0.5f ? 0 : 4;
    } else if (profile_draw >= simple_weight + medium_weight) {
      profile = biome_random01(zone.terrain_seed, 19) < 0.5f ? 2 : 3;
    }
    const float frequency_scale =
        1.0f + std::max(0.0f, config_.terrain_profile_frequency_growth) * zone_difficulty;
    zone.terrain_profile = profile;
    zone.terrain_frequency_scale = frequency_scale;

    zone.jagged_mode = 0;
    zone.jagged_amplitude = 0.0f;
    const int first_sample = std::max(
        0, static_cast<int>(std::ceil(std::max(0.0f, zone.begin_x) / terrain_.dx())));
    const int last_sample = std::min(
        terrain_.sample_count() - 1,
        static_cast<int>(std::floor(std::min(terrain_.length(), zone.end_x) / terrain_.dx())));
    for (int sample = first_sample; sample <= last_sample; ++sample) {
      const float world_x = static_cast<float>(sample) * terrain_.dx();
      const float local_x = world_x - zone.begin_x;

      const float difficulty = terrain_.difficulty_at(world_x);
      const float phase = 6.2831853f * biome_random01(zone.terrain_seed, 17);
      float palette_delta = 0.0f;
      switch (profile) {
        case 0:
          palette_delta = 0.22f * std::sin(local_x * 0.07f * frequency_scale + phase) +
                          0.08f * std::sin(local_x * 0.71f * frequency_scale + phase * 0.37f);
          break;
        case 1: {
          const float cell = 3.8f / frequency_scale;
          const int a = static_cast<int>(std::floor(local_x / cell));
          const float t = local_x / cell - static_cast<float>(a);
          const float va = biome_random01(zone.terrain_seed, 300 + a) * 2.0f - 1.0f;
          const float vb = biome_random01(zone.terrain_seed, 301 + a) * 2.0f - 1.0f;
          const float smooth = t * t * (3.0f - 2.0f * t);
          palette_delta = (va + (vb - va) * smooth) * 0.28f;
          break;
        }
        case 2: {
          const float step = std::floor(local_x / (3.0f / frequency_scale));
          palette_delta = (biome_random01(zone.terrain_seed, 500 + static_cast<uint64_t>(step)) - 0.5f) *
                          0.48f;
          break;
        }
        case 3: {
          const float period = 7.0f / frequency_scale;
          const float f = local_x / period - std::floor(local_x / period);
          palette_delta = (1.0f - 4.0f * std::abs(f - 0.5f)) * 0.38f;
          break;
        }
        default:
          palette_delta = 0.32f * std::sin(local_x * 0.045f * frequency_scale + phase) +
                          (biome_random01(
                               zone.terrain_seed,
                               800 + static_cast<uint64_t>(local_x / (2.0f / frequency_scale))) -
                           0.5f) * 0.16f;
          break;
      }
      const float region_edge = clamp(local_x / 18.0f, 0.0f, 1.0f) *
                                clamp((zone.end_x - world_x) / 18.0f, 0.0f, 1.0f);
      terrain_.add_height_at_index(sample, palette_delta * region_edge *
                                           (0.18f + 0.82f * difficulty));
      if (zone.jagged_mode > 0) {
        float jagged = 0.0f;
        switch (zone.jagged_mode) {
          case 1: {
            const float wavelength = 0.85f + 0.45f * biome_random01(zone.terrain_seed, 63);
            jagged = 0.045f * std::sin(6.2831853f * local_x / wavelength + phase);
            break;
          }
          case 2: {
            jagged = (biome_random01(zone.terrain_seed,
                                     700 + static_cast<uint64_t>(sample)) -
                      0.5f) * 0.11f;
            break;
          }
          case 3: {
            const float wavelength = 1.6f + 1.2f * biome_random01(zone.terrain_seed, 64);
            const float f = local_x / wavelength - std::floor(local_x / wavelength);
            jagged = (f - 0.5f) * 0.14f;
            break;
          }
          default: {
            const float cell = 3.0f;
            const uint64_t block = static_cast<uint64_t>(std::floor(local_x / cell));
            if (biome_random01(zone.terrain_seed, 900 + block) > 0.72f) {
              const float t = local_x / cell - std::floor(local_x / cell);
              jagged = 0.26f * std::sin(3.14159265f * t);
            }
            break;
          }
        }
        terrain_.add_height_at_index(sample, jagged * zone.jagged_amplitude);
      }
      const float delta = biome.terrain_height_delta(local_x, zone.terrain_seed);
      if (std::isfinite(delta)) {
        terrain_.add_height_at_index(
            sample, clamp(delta, -2.5f, 2.5f) * (0.10f + 0.90f * difficulty));
      }
      if (zone.terrain_surprise_mode > 0) {
        const float phase = 6.2831853f * biome_random01(zone.terrain_seed, 91);
        const float entrance = clamp(local_x / 3.0f, 0.0f, 1.0f);
        const float spawn_safe = clamp((world_x - 5.0f) / 4.0f, 0.0f, 1.0f);
        const float remaining = zone.end_x - world_x;
        const float exit = zone.end_x < 999999.0f
                               ? clamp(remaining / 3.0f, 0.0f, 1.0f)
                               : 1.0f;
        const float envelope = entrance * entrance * (3.0f - 2.0f * entrance) *
                               exit * exit * (3.0f - 2.0f * exit) * spawn_safe;
        float surprise = 0.0f;
        switch (zone.terrain_surprise_mode) {
          case 1:
            surprise = 0.42f * std::sin(0.12f * local_x + 0.005f * local_x * local_x + phase);
            break;
          case 2:
            surprise = 0.34f * std::tanh(3.2f * std::sin(0.19f * local_x + phase)) +
                       0.10f * std::sin(0.83f * local_x + phase * 0.37f);
            break;
          case 3: {
            const float ridge = std::max(0.0f, std::sin(0.31f * local_x + phase));
            surprise = 0.62f * ridge * ridge * ridge * ridge - 0.10f;
            break;
          }
          case 4:
            surprise = -0.38f * std::abs(std::sin(0.105f * local_x + phase)) +
                       0.13f * std::sin(1.17f * local_x + phase);
            break;
          default:
            break;
        }
        terrain_.add_height_at_index(
            sample, clamp(surprise * zone.terrain_surprise_strength * envelope, -1.25f, 1.25f));
      }
    }
    const float depth = pending_basin_depth_[static_cast<size_t>(slot)];
    if (depth >= 0.0f) {

      const float field_begin = std::max(0.0f, zone.begin_x) + 8.0f;
      const float field_end =
          std::min(std::min(std::max(terrain_.length(), config_.terrain.length), zone.end_x),
                   field_begin + 620.0f);
      const float level = terrain_.query(field_begin + 6.0f).height;
      zone.liquid_level = level;
      int pond_index = 0;
      for (float pond_begin = field_begin; pond_begin + 26.0f < field_end; ++pond_index) {
        const uint64_t pond_seed = zone.terrain_seed + 7919ULL * static_cast<uint64_t>(pond_index);
        const float length = 42.0f + 48.0f * biome_random01(pond_seed, 31);
        const float pond_end = std::min(field_end, pond_begin + length);

        const int shape = pond_index % 3 == 2 ? 3 : static_cast<int>(
                              biome_random01(pond_seed, 32) * 3.0f) % 3;
        terrain_.carve_pond(pond_begin, pond_end, level,
                            depth * (0.7f + 0.6f * biome_random01(pond_seed, 33)),
                            shape, pond_seed);
        pond_begin = pond_end + 14.0f + 26.0f * biome_random01(pond_seed, 34);
      }
    }
    if (zone.params.ledge_gap_width > 0.0f && zone.params.ledge_spacing > 0.0f) {
      const float zone_start = std::max(zone.begin_x, zone.params.ledge_start_x);

      float last_ledge = std::min(terrain_.length() - 4.0f, zone.end_x - 4.0f);
      if (config_.termination.finish_x > 0.0f) {
        last_ledge = std::min(last_ledge, config_.termination.finish_x - 4.0f);
      }
      for (float begin = zone_start; begin < last_ledge; begin += zone.params.ledge_spacing) {
        const float difficulty = terrain_.difficulty_at(begin);
        if (difficulty < 0.08f) continue;
        const uint64_t ledge_index = static_cast<uint64_t>(std::max(0.0f, std::floor(begin)));
        const bool heavy_ledge = biome_random01(zone.terrain_seed, 700 + ledge_index) > 0.66f;
        const float gap_scale = heavy_ledge
                                    ? 1.65f
                                    : 0.90f + 0.35f * biome_random01(zone.terrain_seed, 900 + ledge_index);
        const float ramp_scale = heavy_ledge
                                     ? 0.70f
                                     : 1.0f + 0.20f * biome_random01(zone.terrain_seed, 1100 + ledge_index);
        const float gap_width = zone.params.ledge_gap_width * gap_scale *
                                (0.25f + 0.75f * difficulty);
        terrain_.carve_ledge(begin, begin + gap_width,
                             zone.params.ledge_ramp_length * ramp_scale,
                             zone.params.ledge_ramp_height * (heavy_ledge ? 1.20f : 1.0f) *
                                 (0.35f + 0.65f * difficulty));
      }
    }
  }
  const auto& first = mechanic_layout_.zones[0];
  mechanic_type_ = first.type;
  mechanic_params_ = first.params;
}

}
