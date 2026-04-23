#pragma once
#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

std::optional<std::pair<int, int>>
parce_record_part(const std::string_view str);

struct GPMFChunkBase;

struct LogEntry {
  enum class Severity { Info, Warning, Error };

  std::chrono::time_point<std::chrono::system_clock> time_;
  Severity severity_;
  std::string message_;
};

struct GPMFParserSettings {
  std::vector<std::string> paths_to_mp4_;
  std::string output_path_;
  std::function<void(LogEntry::Severity, const std::string_view)> log_callback_;
  std::function<void()> end_callback_;
};

class GPMFParser {
public:
  GPMFParser(const GPMFParserSettings &set);
  void parse();
  ~GPMFParser();

private:
  struct impl;
  std::unique_ptr<impl> pimpl_;
};