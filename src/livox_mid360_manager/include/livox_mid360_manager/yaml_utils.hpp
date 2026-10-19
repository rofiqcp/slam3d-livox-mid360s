#pragma once

#include <fstream>
#include <optional>
#include <string>

namespace livox_mid360_manager {

inline std::string trim(std::string s) {
  const auto first = s.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) return {};
  const auto last = s.find_last_not_of(" \t\r\n");
  s = s.substr(first, last - first + 1);
  if (s.size() >= 2 && ((s.front() == '"' && s.back() == '"') ||
                        (s.front() == '\'' && s.back() == '\''))) {
    s = s.substr(1, s.size() - 2);
  }
  return s;
}

inline std::optional<std::string> yaml_scalar(
    const std::string &path, const std::string &section, const std::string &key) {
  std::ifstream in(path);
  if (!in) return std::nullopt;

  std::string line;
  std::string current_section;
  while (std::getline(in, line)) {
    const auto hash = line.find('#');
    if (hash != std::string::npos) line = line.substr(0, hash);
    if (trim(line).empty()) continue;

    const bool indented = !line.empty() && (line[0] == ' ' || line[0] == '\t');
    const auto colon = line.find(':');
    if (colon == std::string::npos) continue;

    const std::string lhs = trim(line.substr(0, colon));
    const std::string rhs = trim(line.substr(colon + 1));

    if (!indented && rhs.empty()) {
      current_section = lhs;
      continue;
    }
    if (current_section == section && lhs == key) return rhs;
  }
  return std::nullopt;
}

}  // namespace livox_mid360_manager
