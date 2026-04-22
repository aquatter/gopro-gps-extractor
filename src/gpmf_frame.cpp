// clang-format off
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <GPMF_mp4reader.h>
// clang-format on
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <filesystem>
#include <fmt/format.h>
#include <gpmf_frame.hpp>
#include <memory>
#include <range/v3/numeric/accumulate.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/filter.hpp>
#include <range/v3/view/iota.hpp>
#include <range/v3/view/sliding.hpp>
#include <range/v3/view/transform.hpp>
#include <range/v3/view/zip.hpp>
#include <stdexcept>
#include <vector>

using ranges::to;
using ranges::views::enumerate;
using ranges::views::filter;
using ranges::views::ints;
using ranges::views::sliding;
using ranges::views::transform;
using ranges::views::zip;

SHUTChunk::SHUTChunk(const SHUTChunkSettings &set) : set_{set} {}

SHUTChunk::~SHUTChunk() = default;

void SHUTChunk::add(const std::string_view, uint64_t timestamp,
                    std::span<const double> vec) {
  data_.emplace_back(timestamp * 1'000, vec.size());
}

void SHUTChunk::increment() { ++index_; }

void SHUTChunk::create_measurements() {}

void SHUTChunk::reset() {
  index_ = 0;
  measurements_.clear();
  data_.clear();
}

void SHUTChunk::open_mp4(const std::string_view path_to_mp4) {}

IMUChunk::IMUChunk() {
  corr_matx_ = Eigen::Matrix4d::Zero();
  corr_matx_(0, 1) = -1.0;
  corr_matx_(1, 0) = 1.0;
  corr_matx_(2, 2) = -1.0;
  corr_matx_(3, 3) = 1.0;

  corr_matx_ = Eigen::Matrix4d::Identity();
}

void IMUChunk::add(const std::string_view fourcc_str, uint64_t timestamp,
                   std::span<const double> vec) {

  Data d{};
  d.timestamp_ = timestamp * 1'000;
  const auto num_elements{vec.size() / num_components_};

  d.val_.resize(num_elements);

  for (auto &&i : ints(0ul, num_elements)) {
    d.val_[i].x() = vec[i * num_components_];
    d.val_[i].y() = vec[i * num_components_ + 1];
    d.val_[i].z() = vec[i * num_components_ + 2];
  }

  if (fourcc_str == "ACCL") {
    accl_data_.push_back(std::move(d));
  } else if (fourcc_str == "GYRO") {
    gyro_data_.push_back(std::move(d));
  }
}

std::vector<IMUChunk::Measurement>
IMUChunk::create_measurements(const std::string_view four_cc) const {

  const auto &data{four_cc == "ACCL" ? accl_data_ : gyro_data_};

  const auto total_num{ranges::accumulate(
      data | transform([](const Data &d) { return d.val_.size(); }), 0ul)};

  std::vector<Measurement> measurements{};
  measurements.reserve(total_num);

  for (auto &&i : ints(0ul, data.size() - 1)) {

    const int64_t delta{data[i + 1].timestamp_ - data[i].timestamp_};

    const double delta_frame{static_cast<double>(delta) /
                             static_cast<double>(data[i].val_.size())};

    for (auto &&[k, val] : enumerate(data[i].val_)) {

      measurements.push_back(Measurement{
          .timestamp_ =
              data[i].timestamp_ + static_cast<int64_t>(k * delta_frame + 0.5),
          .accl_ = (four_cc == "ACCL" ? val : Eigen::Vector3d::Zero()),
          .gyro_ = (four_cc == "GYRO" ? val : Eigen::Vector3d::Zero())});
    }
  }

  const double delta_frame{1'000'000'000.0 / frame_rate_};

  for (auto &&[k, val] : enumerate(data.back().val_)) {
    measurements.push_back(Measurement{
        .timestamp_ = data.back().timestamp_ +
                      static_cast<int64_t>(k * delta_frame + 0.5),
        .accl_ = (four_cc == "ACCL" ? val : Eigen::Vector3d::Zero()),
        .gyro_ = (four_cc == "GYRO" ? val : Eigen::Vector3d::Zero())});
  }

  return measurements;
}

void IMUChunk::create_measurements() {

  auto accl_measurements{create_measurements("ACCL")};
  auto gyro_measurements{create_measurements("GYRO")};

  std::vector<uint8_t> valid_measurements{};
  valid_measurements.resize(gyro_measurements.size(), 0);

  size_t accl_ind{0};
  size_t gyro_ind{0};

  while (accl_ind < accl_measurements.size() and
         gyro_ind < gyro_measurements.size()) {

    while (gyro_ind < gyro_measurements.size() and
           gyro_measurements[gyro_ind].timestamp_ <
               accl_measurements[accl_ind].timestamp_) {
      ++gyro_ind;
    }

    if (gyro_ind == gyro_measurements.size()) {
      break;
    }

    while (accl_ind < accl_measurements.size() and
           accl_measurements[accl_ind].timestamp_ <=
               gyro_measurements[gyro_ind].timestamp_) {
      ++accl_ind;
    }

    if (accl_ind == accl_measurements.size()) {
      break;
    }

    if (accl_ind > 0) {
      if (accl_measurements[accl_ind - 1].timestamp_ ==
          gyro_measurements[gyro_ind].timestamp_) {

        gyro_measurements[gyro_ind].accl_ =
            accl_measurements[accl_ind - 1].accl_;

        valid_measurements[gyro_ind] = 1;
        continue;
      }

      const auto t{
          static_cast<double>(gyro_measurements[gyro_ind].timestamp_ -
                              accl_measurements[accl_ind - 1].timestamp_) /
          static_cast<double>(accl_measurements[accl_ind].timestamp_ -
                              accl_measurements[accl_ind - 1].timestamp_)};

      gyro_measurements[gyro_ind].accl_ =
          accl_measurements[accl_ind - 1].accl_ * (1.0 - t) +
          accl_measurements[accl_ind].accl_ * t;

      valid_measurements[gyro_ind] = 1;
    }
  }

  measurements_ =
      zip(gyro_measurements, valid_measurements) |
      filter([](const auto &val) { return std::get<1>(val) == 1; }) |
      transform([](const auto &val) { return std::get<0>(val); }) |
      to<std::vector>();
}

void IMUChunk::reset() {
  index_ = 0;
  measurements_.clear();
  accl_data_.clear();
  gyro_data_.clear();
}

GPSChunk::GPSChunk(
    std::span<const std::pair<int64_t, int64_t>> exclusion_intervals)
    : exclusion_intervals_{exclusion_intervals.begin(),
                           exclusion_intervals.end()} {}

void GPSChunk::add(const std::string_view, uint64_t timestamp,
                   std::span<const double> vec) {

  Data d{};
  d.timestamp_ = timestamp * 1'000;
  const auto num_elements{vec.size() / num_components_};

  d.lla_.resize(num_elements);
  d.vel2d_.resize(num_elements);
  d.vel3d_.resize(num_elements);
  d.fix_.resize(num_elements);

  for (auto &&i : ints(0ul, num_elements)) {
    d.lla_[i].x() = vec[i * num_components_];
    d.lla_[i].y() = vec[i * num_components_ + 1];
    d.lla_[i].z() = vec[i * num_components_ + 2];
    d.vel2d_[i] = vec[i * num_components_ + 3];
    d.vel3d_[i] = vec[i * num_components_ + 4];
    d.dop_[i] = vec[i * num_components_ + 7];
    d.fix_[i] = vec[i * num_components_ + 8];
  }

  data_.push_back(std::move(d));
}

void GPSChunk::create_measurements() {

  const auto total_num{ranges::accumulate(
      data_ | transform([](const Data &d) { return d.lla_.size(); }), 0ul)};

  measurements_.reserve(total_num);

  for (auto &&d : data_ | sliding(2)) {
    const int64_t delta{d[1].timestamp_ - d[0].timestamp_};
    const double delta_frame{static_cast<double>(delta) /
                             static_cast<double>(d[0].lla_.size())};

    for (auto &&[k, gps_data] : enumerate(
             zip(d[0].lla_, d[0].vel2d_, d[0].vel3d_, d[0].dop_, d[0].fix_))) {
      auto &&[lla, vel2d, vel3d, dop, fix] = gps_data;

      measurements_.push_back(
          Measurement{.timestamp_ = d[0].timestamp_ +
                                    static_cast<int64_t>(k * delta_frame + 0.5),
                      .lla_ = lla,
                      .vel2d_ = vel2d,
                      .vel3d_ = vel3d,
                      .dop_ = dop,
                      .fix_ = fix});
    }
  }

  const double delta_frame{1'000'000'000.0 / frame_rate_};

  for (auto &&[k, gps_data] : enumerate(
           zip(data_.back().lla_, data_.back().vel2d_, data_.back().vel3d_,
               data_.back().dop_, data_.back().fix_))) {
    auto &&[lla, vel2d, vel3d, dop, fix] = gps_data;

    measurements_.push_back(
        Measurement{.timestamp_ = data_.back().timestamp_ +
                                  static_cast<int64_t>(k * delta_frame + 0.5),
                    .lla_ = lla,
                    .vel2d_ = vel2d,
                    .vel3d_ = vel3d,
                    .dop_ = dop,
                    .fix_ = fix});
  }
}

bool GPSChunk::exclude(int64_t timestamp) const noexcept {
  for (auto &&[start, end] : exclusion_intervals_) {
    if (timestamp >= start and timestamp <= end) {
      return true;
    }
  }

  return false;
}