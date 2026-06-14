// gnc-sim — pure serialization of SimResult. NO file I/O (keeps core WASM-safe):
//  - toJsonString:  columnar JSON for the web app (Plotly-friendly arrays).
//  - toCsvFiles:    map of {filename -> CSV text}; the native CLI writes these to disk.
//  - toManifestJson: run metadata (seed, git sha, origin, miss distance...).
#pragma once

#include <map>
#include <string>

#include "gncsim/core/Types.hpp"

namespace gncsim {

std::string toJsonString(const SimResult& r, int indent = -1);
std::map<std::string, std::string> toCsvFiles(const SimResult& r);
std::string toManifestJson(const SimResult& r, const std::string& config_echo = "");

}  // namespace gncsim
