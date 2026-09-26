#pragma once

#include <cstddef>

#include "mars/state.hpp"

namespace mars {

constexpr int kTerrainSamplesAhead = 24;
constexpr int kBiomeSamplesAhead = 8;
constexpr int kImuObservationDim = 4;
constexpr int kBodyContactObservationDim = 3;
constexpr int kWheelEncoderObservationDim = kMaxWheels * 2;
constexpr int kObservationDim =
    8 +
    kMaxWheels * 3 +
    kTerrainSamplesAhead * 2 +
    kBiomeSamplesAhead * 3 +
    18 +
    kImuObservationDim +
    kBodyContactObservationDim +
    kWheelEncoderObservationDim + 15;

struct ObservationView {
  float* data = nullptr;
  int dim = kObservationDim;
};

}
