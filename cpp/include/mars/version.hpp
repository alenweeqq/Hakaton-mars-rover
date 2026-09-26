#pragma once

#include <string_view>

namespace mars {

inline constexpr std::string_view kRepositoryVersion = "0.16.0";
inline constexpr std::string_view kEnvironmentVersion = kRepositoryVersion;
#ifdef MARS_ROVER_BANK_VERSION
inline constexpr std::string_view kBiomeBankVersion = MARS_ROVER_BANK_VERSION;
#else
inline constexpr std::string_view kBiomeBankVersion = "sha256:unfingerprinted";
#endif

}
