#include "mars/terrain.hpp"

#include <algorithm>
#include <cmath>
#include <random>

namespace mars {

namespace {

float seeded_unit(uint64_t seed, uint64_t stream) {
  uint64_t x = seed + 0x9e3779b97f4a7c15ULL * (stream + 1ULL);
  x = (x ^ (x >> 30U)) * 0xbf58476d1ce4e5b9ULL;
  x = (x ^ (x >> 27U)) * 0x94d049bb133111ebULL;
  x ^= x >> 31U;
  return static_cast<float>(x >> 40U) * (1.0f / 16777216.0f);
}

}

void Terrain::configure(const TerrainConfig& config) {
  dx_ = config.dx;
  inv_dx_ = 1.0f / config.dx;
  base_height_ = config.base_height;
  amplitude_ = config.amplitude;
  roughness_ = config.roughness;
  crater_count_ = config.crater_count;
  step_count_ = config.step_count;
  safe_start_fraction_ = clamp(config.safe_start_fraction, 0.0f, 0.45f);
  difficulty_exponent_ = clamp(config.difficulty_exponent, 0.5f, 4.0f);
  difficulty_distance_offset_ = std::max(0.0f, config.difficulty_distance_offset);
  preserve_spawn_safety_ = config.preserve_spawn_safety;
  heights_.assign(static_cast<size_t>(config.sample_count), config.base_height);
  solid_.assign(static_cast<size_t>(config.sample_count), 1u);
  surfaces_.clear();
}

void Terrain::generate(uint64_t seed) {
  std::mt19937_64 rng(seed);
  std::uniform_real_distribution<float> phase_dist(0.0f, 6.2831853f);
  std::uniform_real_distribution<float> unit_dist(0.0f, 1.0f);
  const float p0 = phase_dist(rng);
  const float p1 = phase_dist(rng);
  const float p2 = phase_dist(rng);
  for (int i = 0; i < static_cast<int>(heights_.size()); ++i) {
    const float x = static_cast<float>(i) * dx_;
    const float difficulty = difficulty_at(x);
    const float profile_scale = 0.18f + 0.82f * difficulty;
    heights_[static_cast<size_t>(i)] =
        base_height_ +
        amplitude_ * profile_scale * (0.45f * std::sin(0.13f * x + p0) +
                      0.22f * std::sin(0.53f * x + p1) +
                      0.10f * std::sin(1.37f * x + p2));
  }

  const float max_x = length();
  const float safe_end = max_x * safe_start_fraction_;
  const auto progressive_position = [&]() {
    const float mixture = unit_dist(rng);
    const float u = unit_dist(rng);


    const float t = mixture < 0.35f ? u : std::pow(u, 0.58f);
    return safe_end + 2.0f + t * std::max(1.0f, max_x - safe_end - 6.0f);
  };
  generated_step_count_ = std::max(0, step_count_);
  for (int n = 0; n < generated_step_count_; ++n) {
    const float x0 = progressive_position();
    const float difficulty = difficulty_at(x0);
    const float width = 0.45f + (0.35f + 2.7f * difficulty) * unit_dist(rng);
    const float severity = 0.10f + 0.90f * difficulty;
    const float height = (unit_dist(rng) * 2.0f - 1.0f) * amplitude_ *
                         (0.35f + roughness_) * severity;
    for (int i = 0; i < static_cast<int>(heights_.size()); ++i) {
      const float x = static_cast<float>(i) * dx_;
      const float t = clamp((x - x0) / width, 0.0f, 1.0f);
      const float smooth = t * t * (3.0f - 2.0f * t);
      heights_[static_cast<size_t>(i)] += height * smooth;
    }
  }

  generated_pit_count_ = std::max(0, crater_count_);
  for (int n = 0; n < generated_pit_count_; ++n) {
    const float cx = progressive_position();
    const float difficulty = difficulty_at(cx);


    const float pit_draw = unit_dist(rng);
    const bool deep_pit = pit_draw < 0.02f + 0.36f * difficulty * difficulty;
    const bool broad_pit = !deep_pit && pit_draw < 0.24f + 0.52f * difficulty;



    const float radius = deep_pit
        ? 0.58f + unit_dist(rng) * (0.22f + 0.30f * difficulty)
        : broad_pit
            ? 1.20f + difficulty * (1.15f + unit_dist(rng) * 2.60f)
            : 0.55f + difficulty * (0.85f + unit_dist(rng) * 3.1f);
    const float depth = deep_pit
        ? 1.50f + difficulty * (0.55f + unit_dist(rng) * 1.10f)
        : broad_pit
            ? amplitude_ * (0.24f + difficulty * (0.42f + 0.58f * unit_dist(rng)))
            : amplitude_ * (0.08f + difficulty * (0.45f + 0.85f * unit_dist(rng)));
    for (int i = 0; i < static_cast<int>(heights_.size()); ++i) {
      const float x = static_cast<float>(i) * dx_;
      const float d = std::abs(x - cx) / radius;
      if (d < 1.0f) {
        const float bowl = 0.5f + 0.5f * std::cos(d * 3.14159265f);
        heights_[static_cast<size_t>(i)] -= depth * bowl;
      } else if (d < 1.45f) {
        const float rim_t = (d - 1.0f) / 0.45f;
        heights_[static_cast<size_t>(i)] += depth * 0.22f * (1.0f - rim_t);
      }
    }
  }

  for (int pass = 0; pass < 2; ++pass) {
    float prev = heights_.front();
    for (int i = 1; i < static_cast<int>(heights_.size()) - 1; ++i) {
      const float current = heights_[static_cast<size_t>(i)];
      heights_[static_cast<size_t>(i)] =
          prev * 0.18f + current * 0.64f + heights_[static_cast<size_t>(i + 1)] * 0.18f;
      prev = current;
    }
  }

  if (preserve_spawn_safety_) {
    const int spawn_safe_samples = std::min(static_cast<int>(heights_.size()), static_cast<int>(5.0f / dx_));
    for (int i = 0; i < spawn_safe_samples; ++i) {
      const float t = static_cast<float>(i) /
                      std::max(1.0f, static_cast<float>(spawn_safe_samples - 1));
      heights_[static_cast<size_t>(i)] =
          base_height_ * (1.0f - t) + heights_[static_cast<size_t>(i)] * t;
    }
  }
}

float Terrain::difficulty_at(float x) const {
  const float course_length = std::max(dx_, length());
  const float progress = clamp((x + difficulty_distance_offset_) / course_length, 0.0f, 1.0f);
  if (progress <= safe_start_fraction_) return 0.0f;
  const float t = clamp((progress - safe_start_fraction_) /
                            std::max(0.01f, 1.0f - safe_start_fraction_),
                        0.0f, 1.0f);
  const float broad = std::pow(t, difficulty_exponent_);
  const float late_curve = std::pow(t, difficulty_exponent_ * 2.35f);
  return clamp(0.68f * broad + 0.32f * late_curve, 0.0f, 1.0f);
}

TerrainSample Terrain::query(float x) const {
  if (heights_.empty()) {
    return {};
  }
  const int last = static_cast<int>(heights_.size()) - 1;
  const float fx = clamp(x * inv_dx_, 0.0f, static_cast<float>(last));
  const int i0 = static_cast<int>(std::floor(fx));
  const int i1 = std::min(i0 + 1, last);
  if (solid_[static_cast<size_t>(i0)] == 0u || solid_[static_cast<size_t>(i1)] == 0u) {
    return {-1000.0f, 0.0f, {0.0f, 1.0f}, false};
  }
  float h, slope;
  query_height_slope(x, h, slope);
  const float inv_len = 1.0f / std::sqrt(1.0f + slope * slope);
  return {h, slope, {-slope * inv_len, inv_len}, true};
}

TerrainSample Terrain::query_near(float x, float reference_y) const {
  TerrainSample result = query(x);
  float selected_height = result.solid ? result.height : -1.0e9f;


  for (const auto& surface : surfaces_) {
    if (x < surface.begin_x || x > surface.end_x) continue;
    const float t = (x - surface.begin_x) /
                    std::max(0.0001f, surface.end_x - surface.begin_x);
    const float h = surface.begin_height + (surface.end_height - surface.begin_height) * t;
    if (h <= reference_y + 0.08f && h > selected_height) {
      selected_height = h;
      result.height = h;
      result.slope = (surface.end_height - surface.begin_height) /
                     std::max(0.0001f, surface.end_x - surface.begin_x);
      const float inv_len = 1.0f / std::sqrt(1.0f + result.slope * result.slope);
      result.normal = {-result.slope * inv_len, inv_len};
      result.solid = true;
    }
  }
  return result;
}

void Terrain::query_height_slope(float x, float& h, float& slope) const {
  if (heights_.empty()) {
    h = 0.0f;
    slope = 0.0f;
    return;
  }
  const int last = static_cast<int>(heights_.size()) - 1;
  const float fx = clamp(x * inv_dx_, 0.0f, static_cast<float>(last));
  const int i0 = static_cast<int>(std::floor(fx));
  const int i1 = std::min(i0 + 1, last);
  const float t = fx - static_cast<float>(i0);
  h = heights_[static_cast<size_t>(i0)] * (1.0f - t) + heights_[static_cast<size_t>(i1)] * t;
  const int im = std::max(i0 - 1, 0);
  const int ip = std::min(i0 + 1, last);
  slope = (heights_[static_cast<size_t>(ip)] - heights_[static_cast<size_t>(im)]) /
          (static_cast<float>(ip - im) * dx_ + 1.0e-6f);
}

void Terrain::deform(float x, float radius, float amount) {
  if (heights_.empty() || radius <= 0.0f) {
    return;
  }
  const int first = std::max(0, static_cast<int>(std::floor((x - radius) / dx_)));
  const int last = std::min(static_cast<int>(heights_.size() - 1),
                            static_cast<int>(std::ceil((x + radius) / dx_)));
  for (int i = first; i <= last; ++i) {
    const float sx = static_cast<float>(i) * dx_;
    const float d = std::abs(sx - x) / radius;
    const float w = clamp(1.0f - d, 0.0f, 1.0f);
    if (solid_[static_cast<size_t>(i)] != 0u) {
      heights_[static_cast<size_t>(i)] -= amount * w;
    }
  }
}

void Terrain::add_height_at_index(int index, float amount) {
  if (index < 0 || index >= static_cast<int>(heights_.size()) || !std::isfinite(amount)) return;
  heights_[static_cast<size_t>(index)] += amount;
}

float Terrain::carve_basin(float begin_x, float end_x, float depth, uint64_t seed) {
  if (heights_.empty() || end_x <= begin_x || depth <= 0.0f) {
    return base_height_;
  }
  const float left_height = query(begin_x).height;
  const float right_height = query(end_x).height;
  const float water_level = std::min(left_height, right_height);
  const int first = std::max(0, static_cast<int>(std::floor(begin_x * inv_dx_)));
  const int last = std::min(static_cast<int>(heights_.size() - 1),
                            static_cast<int>(std::ceil(end_x * inv_dx_)));
  const float inv_width = 1.0f / (end_x - begin_x);
  const int profile = static_cast<int>(seeded_unit(seed, 40) * 4.0f) % 4;
  const float phase_a = seeded_unit(seed, 41) * 6.2831853f;
  const float phase_b = seeded_unit(seed, 42) * 6.2831853f;
  const float skew = 0.72f + 0.56f * seeded_unit(seed, 43);
  for (int i = first; i <= last; ++i) {
    const float x = static_cast<float>(i) * dx_;
    const float t = clamp((x - begin_x) * inv_width, 0.0f, 1.0f);
    const float bowl = std::sin(t * 3.14159265f);
    const float bank = left_height * (1.0f - t) + right_height * t;
    const float envelope = bowl * bowl;
    float floor_shape = 1.0f;
    switch (profile) {
      case 0:
        floor_shape = 0.92f + 0.08f * std::cos((t - 0.5f) * 3.14159265f);
        break;
      case 1:
        floor_shape = 0.78f + 0.28f * std::pow(t, skew);
        break;
      case 2:
        floor_shape = 0.88f + 0.12f * std::sin(t * 6.2831853f + phase_a) +
                      0.05f * std::sin(t * 18.8495559f + phase_b);
        break;
      default: {
        const float cells = 3.0f + std::floor(seeded_unit(seed, 44) * 3.0f);
        const float local = t * cells - std::floor(t * cells);
        const float rounded = local * local * (3.0f - 2.0f * local);
        floor_shape = 0.78f + 0.18f * rounded + 0.06f * std::sin(t * 3.14159265f);
        break;
      }
    }
    heights_[static_cast<size_t>(i)] =
        bank - depth * envelope * clamp(floor_shape, 0.62f, 1.12f);
  }
  return water_level;
}

void Terrain::carve_pond(float begin_x, float end_x, float water_level, float depth,
                         int shape, uint64_t seed) {
  if (heights_.empty() || end_x <= begin_x || depth <= 0.0f) return;
  const int first = std::max(0, static_cast<int>(std::floor(begin_x * inv_dx_)));
  const int last = std::min(static_cast<int>(heights_.size() - 1),
                            static_cast<int>(std::ceil(end_x * inv_dx_)));
  const float inv_width = 1.0f / (end_x - begin_x);
  const float terraces = 3.0f + std::floor(seeded_unit(seed, 51) * 3.0f);
  for (int i = first; i <= last; ++i) {
    const float x = static_cast<float>(i) * dx_;
    const float t = clamp((x - begin_x) * inv_width, 0.0f, 1.0f);
    const float edge = std::min(t, 1.0f - t);
    float sink = 0.0f;
    switch (shape) {
      case 0: {
        const float bowl = std::sin(t * 3.14159265f);
        sink = bowl * bowl;
        break;
      }
      case 1: {
        const float shore = 0.26f;
        const float ramp = clamp(edge / shore, 0.0f, 1.0f);
        sink = ramp * ramp * (3.0f - 2.0f * ramp);
        break;
      }
      case 2: {
        const float ramp = clamp(edge / 0.34f, 0.0f, 1.0f);
        const float scaled = ramp * terraces;
        const float step = std::floor(scaled);
        const float local = scaled - step;
        const float eased = local * local * (3.0f - 2.0f * local);
        sink = (step + eased) / terraces;
        break;
      }
      default: {
        const float shore = 0.05f;
        sink = clamp(edge / shore, 0.0f, 1.0f);
        break;
      }
    }
    const float floor_height = water_level - depth * sink;
    heights_[static_cast<size_t>(i)] =
        std::min(heights_[static_cast<size_t>(i)], floor_height);
    if (shape == 3) {


      const float lip = (t - 0.94f) / 0.03f;
      if (std::abs(lip) < 1.0f) {
        const float crest = water_level + 0.55f * (1.0f - lip * lip);
        heights_[static_cast<size_t>(i)] =
            std::max(heights_[static_cast<size_t>(i)], crest);
      }
    }
  }
}

void Terrain::carve_ledge(float begin_x, float end_x, float ramp_length,
                          float ramp_height) {
  if (heights_.empty() || end_x <= begin_x) return;
  const float run = std::max(dx_ * 2.0f, ramp_length);
  const auto smoothstep = [](float t) { return t * t * (3.0f - 2.0f * t); };




  const float ramp_begin = std::max(0.0f, begin_x - run);
  const int ramp_first = std::max(0, static_cast<int>(std::floor(ramp_begin * inv_dx_)));
  const int ramp_last = std::min(static_cast<int>(heights_.size() - 1),
                                 static_cast<int>(std::floor(begin_x * inv_dx_)));
  for (int i = ramp_first; i <= ramp_last; ++i) {
    const float x = static_cast<float>(i) * dx_;
    const float t = clamp((x - ramp_begin) / std::max(dx_, begin_x - ramp_begin), 0.0f, 1.0f);
    const float lift = t < 0.78f ? smoothstep(t / 0.78f) : 1.0f;
    heights_[static_cast<size_t>(i)] += ramp_height * lift;
  }

  const int first = std::max(0, static_cast<int>(std::ceil(begin_x * inv_dx_)));
  const int last = std::min(static_cast<int>(heights_.size() - 1),
                            static_cast<int>(std::floor(end_x * inv_dx_)));
  for (int i = first; i <= last; ++i) solid_[static_cast<size_t>(i)] = 0u;



  const float landing_run = run * 1.6f;
  const int landing_first = std::max(0, static_cast<int>(std::ceil(end_x * inv_dx_)));
  const int landing_last = std::min(static_cast<int>(heights_.size() - 1),
                                    static_cast<int>(std::floor((end_x + landing_run) * inv_dx_)));
  for (int i = landing_first; i <= landing_last; ++i) {
    const float x = static_cast<float>(i) * dx_;
    const float t = clamp((x - end_x) / std::max(dx_, landing_run), 0.0f, 1.0f);
    heights_[static_cast<size_t>(i)] += ramp_height * (1.0f - smoothstep(t));
  }
}

void Terrain::add_surface(float begin_x, float end_x, float begin_height, float end_height) {
  if (end_x - begin_x < dx_ || !std::isfinite(begin_height) || !std::isfinite(end_height)) return;
  surfaces_.push_back({begin_x, end_x, begin_height, end_height});
}

float Terrain::next_solid_x(float x, float required_run) const {
  if (heights_.empty()) return x;
  const float run = std::max(dx_, required_run);
  const int run_samples = std::max(1, static_cast<int>(std::ceil(run * inv_dx_)));
  const int first = std::clamp(static_cast<int>(std::floor(x * inv_dx_)), 0,
                               static_cast<int>(heights_.size() - 1));
  for (int i = first; i + run_samples < static_cast<int>(solid_.size()); ++i) {
    bool stable = true;
    for (int j = 0; j <= run_samples; ++j) {
      if (solid_[static_cast<size_t>(i + j)] == 0u) {
        stable = false;
        break;
      }
    }
    if (stable) return static_cast<float>(i) * dx_ + 0.5f * run;
  }
  return std::min(length(), std::max(0.0f, x) + run);
}

float Terrain::previous_solid_x(float x, float required_run) const {
  if (heights_.empty()) return x;
  const float run = std::max(dx_, required_run);
  const int run_samples = std::max(1, static_cast<int>(std::ceil(run * inv_dx_)));
  const int last = static_cast<int>(solid_.size()) - 1;
  const int first = std::clamp(static_cast<int>(std::floor(x * inv_dx_)), 0, last);
  for (int i = first; i - run_samples >= 0; --i) {
    bool stable = true;
    for (int j = 0; j <= run_samples; ++j) {
      if (solid_[static_cast<size_t>(i - j)] == 0u) {
        stable = false;
        break;
      }
    }
    if (stable) return static_cast<float>(i) * dx_ - 0.5f * run;
  }
  return next_solid_x(0.0f, run);
}

float Terrain::height_at_index(int i) const {
  return heights_[static_cast<size_t>(std::clamp(i, 0, static_cast<int>(heights_.size() - 1)))];
}

}
