#pragma once

#include <Eigen/Core>
#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <string_view>
#include <utility>
#include <vector>

enum class ChunkType { Camera, GPS, IMU };

struct GPMFChunkBase {

  virtual void add(const std::string_view fourcc_str, uint64_t timestamp,
                   std::span<const double> vec) = 0;

  virtual std::span<const std::string_view> four_cc() const = 0;
  virtual ChunkType whoami() const = 0;
  virtual void create_measurements() = 0;

  virtual ~GPMFChunkBase() = default;

  virtual std::optional<int64_t> timestamp() = 0;
  virtual std::pair<int64_t, int64_t> timestamp_range() const = 0;

  virtual void reset() = 0;

  void set_frame_rate(double frame_rate) noexcept { frame_rate_ = frame_rate; }

  virtual void increment() { ++index_; };

  void visit(std::function<void(const GPMFChunkBase *)> f) const { f(this); }

  double frame_rate_;
  size_t index_{0};
};

struct SHUTChunkSettings {
  double resize_;
  std::string output_path_;
  int jpeg_quality_;
  bool extract_images_;
  bool compress_;
  bool fix_distortion_;
  std::string calibration_path_;
};

struct SHUTChunk : GPMFChunkBase {

  SHUTChunk(const SHUTChunkSettings &set);

  struct Data {
    int64_t timestamp_;
    size_t num_frames_;
  };

  std::span<const std::string_view> four_cc() const override {
    return fourcc_str_;
  }

  ChunkType whoami() const override { return ChunkType::Camera; }

  void add(const std::string_view, uint64_t timestamp,
           std::span<const double> vec) override;

  void create_measurements() override;

  std::optional<int64_t> timestamp() override {
    if (index_ < measurements_.size()) {
      return measurements_[index_];
    }

    return std::nullopt;
  }

  std::pair<int64_t, int64_t> timestamp_range() const override {
    return {measurements_.front(), measurements_.back()};
  }

  void increment() override;

  void reset() override;

  ~SHUTChunk();

  static constexpr std::array<std::string_view, 1> fourcc_str_{"SHUT"};
  std::vector<Data> data_;

  SHUTChunkSettings set_;

  std::vector<int64_t> measurements_;
  size_t num_frames_;
};

struct GPSChunk : GPMFChunkBase {

  struct Data {
    int64_t timestamp_;
    std::vector<Eigen::Vector3d> lla_;
    std::vector<double> vel2d_;
    std::vector<double> vel3d_;
    std::vector<double> dop_;
    std::vector<double> fix_;
  };

  struct Measurement {
    int64_t timestamp_;
    Eigen::Vector3d lla_;
    double vel2d_;
    double vel3d_;
    double dop_;
    double fix_;
  };

  GPSChunk() = default;

  std::span<const std::string_view> four_cc() const override {
    return fourcc_str_;
  }

  ChunkType whoami() const override { return ChunkType::GPS; }

  void add(const std::string_view, uint64_t timestamp,
           std::span<const double> vec) override;

  void create_measurements() override;

  std::optional<int64_t> timestamp() override {
    if (index_ < measurements_.size()) {
      return measurements_[index_].timestamp_;
    }

    return std::nullopt;
  }

  std::pair<int64_t, int64_t> timestamp_range() const override {
    return {measurements_.front().timestamp_, measurements_.back().timestamp_};
  }

  void reset() override;

  static constexpr size_t num_components_{9};
  static constexpr std::array<std::string_view, 1> fourcc_str_{"GPS9"};
  std::vector<Data> data_;
  std::vector<Measurement> measurements_;
};

struct IMUChunk : GPMFChunkBase {
  IMUChunk();

  struct Data {
    int64_t timestamp_;
    std::vector<Eigen::Vector3d> val_;
  };

  struct Measurement {
    int64_t timestamp_;
    Eigen::Vector3d accl_;
    Eigen::Vector3d gyro_;
  };

  std::span<const std::string_view> four_cc() const override {
    return fourcc_str_;
  }

  ChunkType whoami() const override { return ChunkType::IMU; }

  void add(const std::string_view fourcc_str, uint64_t timestamp,
           std::span<const double> vec) override;

  void create_measurements() override;

  std::vector<Measurement>
  create_measurements(const std::string_view four_cc) const;

  std::optional<int64_t> timestamp() override {
    if (index_ < measurements_.size()) {
      return measurements_[index_].timestamp_;
    }

    return std::nullopt;
  }

  void reset() override;

  std::pair<int64_t, int64_t> timestamp_range() const override {
    return {measurements_.front().timestamp_, measurements_.back().timestamp_};
  }

  static constexpr size_t num_components_{3};
  static constexpr std::array<std::string_view, 2> fourcc_str_{"ACCL", "GYRO"};

  std::vector<Data> accl_data_;
  std::vector<Data> gyro_data_;
  std::vector<Measurement> measurements_;
  Eigen::Matrix4d corr_matx_;
};
