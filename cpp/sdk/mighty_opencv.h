#pragma once

#include <algorithm>
#include <cctype>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "mighty_client.h"

namespace mighty_protocol {
namespace sdk {
namespace opencv {

struct DecodedImageFrame {
  cv::Mat bgr;
  uint64_t timestamp_ns = 0;
  std::string channel;
  std::string channel_alias;
  bool is_reference = false;
};

inline bool is_primary_channel(const RawImageFrame& image) {
  std::string channel =
      image.channel_alias.empty() ? image.channel : image.channel_alias;
  std::transform(
      channel.begin(), channel.end(), channel.begin(),
      [](unsigned char value) { return static_cast<char>(std::tolower(value)); });
  return channel.empty() || channel == "cam0" || channel == "preview" ||
         channel == "left";
}

inline const RawImageFrame* select_primary_raw(const ImageFrame& image) {
  if (image.kind == ImageFrame::Kind::kRaw) return &image.left;
  if (image.kind != ImageFrame::Kind::kStereoRaw) return nullptr;
  if (is_primary_channel(image.left)) return &image.left;
  if (image.right && is_primary_channel(*image.right)) return &*image.right;
  return &image.left;
}

inline bool decode_raw_to_bgr(const RawImageFrame& raw, cv::Mat* output) {
  if (!output || raw.width == 0 || raw.height == 0) return false;
  const int width = static_cast<int>(raw.width);
  const int height = static_cast<int>(raw.height);
  const std::size_t pixels =
      static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  const auto format = static_cast<RawFormat>(raw.format);

  if (format == RawFormat::kGray8 || format == RawFormat::kYUV420P ||
      format == RawFormat::kYUV420SP) {
    if (raw.data.size() < pixels) return false;
    const cv::Mat gray(height, width, CV_8UC1,
                       const_cast<uint8_t*>(raw.data.data()));
    cv::cvtColor(gray, *output, cv::COLOR_GRAY2BGR);
    return true;
  }

  int channels = 0;
  int conversion = -1;
  if (format == RawFormat::kBGR24) {
    channels = 3;
  } else if (format == RawFormat::kRGB24) {
    channels = 3;
    conversion = cv::COLOR_RGB2BGR;
  } else if (format == RawFormat::kBGRA32) {
    channels = 4;
    conversion = cv::COLOR_BGRA2BGR;
  } else if (format == RawFormat::kRGBA32) {
    channels = 4;
    conversion = cv::COLOR_RGBA2BGR;
  } else {
    return false;
  }
  if (raw.data.size() < pixels * static_cast<std::size_t>(channels)) return false;
  const cv::Mat source(height, width, CV_MAKETYPE(CV_8U, channels),
                       const_cast<uint8_t*>(raw.data.data()));
  if (conversion < 0) {
    *output = source.clone();
  } else {
    cv::cvtColor(source, *output, conversion);
  }
  return !output->empty();
}

inline bool decode_jpeg_to_bgr(const JpegImageFrame& jpeg, cv::Mat* output) {
  if (!output || jpeg.data.empty()) return false;
  const cv::Mat encoded(1, static_cast<int>(jpeg.data.size()), CV_8UC1,
                        const_cast<uint8_t*>(jpeg.data.data()));
  *output = cv::imdecode(encoded, cv::IMREAD_COLOR);
  return !output->empty();
}

inline bool decode_image_to_bgr(const ImageFrame& image,
                                DecodedImageFrame* output,
                                bool include_reference = false) {
  if (!output) return false;
  if (image.kind == ImageFrame::Kind::kJpeg) {
    if (image.jpeg && image.jpeg->is_reference && !include_reference) return false;
    if (!image.jpeg || !decode_jpeg_to_bgr(*image.jpeg, &output->bgr)) return false;
    output->timestamp_ns = image.jpeg->timestamp_ns;
    output->channel = image.jpeg->channel;
    output->channel_alias = image.jpeg->channel_alias;
    output->is_reference = image.jpeg->is_reference;
    return true;
  }
  const RawImageFrame* raw = select_primary_raw(image);
  if (!raw || !decode_raw_to_bgr(*raw, &output->bgr)) return false;
  output->timestamp_ns = raw->timestamp_ns;
  output->channel = raw->channel;
  output->channel_alias = raw->channel_alias;
  output->is_reference = false;
  return true;
}

inline bool decode_jpeg_to_gray8_raw(const JpegImageFrame& jpeg,
                                     RawImageFrame* output) {
  if (!output || jpeg.data.empty()) return false;
  const cv::Mat encoded(1, static_cast<int>(jpeg.data.size()), CV_8UC1,
                        const_cast<uint8_t*>(jpeg.data.data()));
  const cv::Mat gray = cv::imdecode(encoded, cv::IMREAD_GRAYSCALE);
  if (gray.empty()) return false;
  cv::Mat packed = gray.isContinuous() ? gray : gray.clone();
  output->timestamp_ns = jpeg.timestamp_ns;
  output->width = static_cast<uint32_t>(packed.cols);
  output->height = static_cast<uint32_t>(packed.rows);
  output->format = static_cast<uint8_t>(RawFormat::kGray8);
  output->channel = jpeg.channel;
  output->channel_alias = jpeg.channel_alias;
  output->data.assign(packed.data, packed.data + packed.total());
  return true;
}

inline const RawImageFrame* image_to_raw(const ImageFrame& image,
                                         RawImageFrame* decoded_jpeg,
                                         bool include_reference = false) {
  if (image.kind == ImageFrame::Kind::kJpeg) {
    if (image.jpeg && image.jpeg->is_reference && !include_reference) {
      return nullptr;
    }
    if (!image.jpeg ||
        !decode_jpeg_to_gray8_raw(*image.jpeg, decoded_jpeg)) {
      return nullptr;
    }
    return decoded_jpeg;
  }
  return select_primary_raw(image);
}

}  // namespace opencv
}  // namespace sdk
}  // namespace mighty_protocol
