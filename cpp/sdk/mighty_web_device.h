#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "mighty_device_io.h"
#include "../third_party/httplib.h"

namespace mighty_protocol {
namespace sdk {

struct MightyWebDeviceOptions {
  std::string base_url;
  std::vector<std::string> base_urls;
  std::string stream_path = "/stream";
  std::string command_path = "/command";
  std::map<std::string, std::string> headers;
  int connect_timeout_ms = 5000;
  int read_timeout_ms = 1000;
  int write_timeout_ms = 5000;
};

class MightyWebDevice : public MightyDeviceIO {
 public:
  MightyWebDevice() : MightyWebDevice(MightyWebDeviceOptions()) {}

  explicit MightyWebDevice(const MightyWebDeviceOptions& options)
      : options_(options) {
    std::string error;
    if (!build_endpoints(options_, &endpoints_, &error)) {
      init_error_ = error;
      return;
    }
    is_valid_ = true;
  }

  bool connect(BytesCallback on_bytes, std::string* error) override {
    if (!on_bytes) {
      if (error) *error = "connect requires a valid bytes callback";
      return false;
    }

    {
      std::lock_guard<std::mutex> lock(state_mu_);
      if (connected_) {
        if (error) *error = "stream already connected";
        return false;
      }
      connected_ = true;
      stop_requested_ = false;
    }

    if (!is_valid_) {
      std::lock_guard<std::mutex> lock(state_mu_);
      connected_ = false;
      if (error) *error = init_error_;
      return false;
    }

    std::string last_error = "stream request failed (no host)";
    const std::vector<size_t> indices = ordered_endpoint_indices();
    for (size_t i : indices) {
      std::string attempt_error;
      if (connect_to_endpoint(i, on_bytes, &attempt_error)) {
        std::lock_guard<std::mutex> lock(state_mu_);
        active_endpoint_index_ = static_cast<int>(i);
        return true;
      }
      if (should_stop()) {
        return true;
      }
      if (!attempt_error.empty()) last_error = attempt_error;
    }

    {
      std::lock_guard<std::mutex> lock(state_mu_);
      connected_ = false;
    }
    if (error) *error = last_error;
    return false;
  }

  void disconnect() override {
    std::lock_guard<std::mutex> lock(state_mu_);
    stop_requested_ = true;
    if (active_stream_client_ != nullptr) {
      active_stream_client_->stop();
    }
  }

  bool send_command_payload(const std::vector<uint8_t>& cmd_payload,
                            std::vector<uint8_t>* response_payload,
                            std::string* error) override {
    if (!is_valid_) {
      if (error) *error = init_error_;
      return false;
    }

    const std::vector<size_t> indices = ordered_endpoint_indices();
    std::string last_error = "command request failed (no host)";

    for (size_t i : indices) {
      const Endpoint& endpoint = endpoints_[i];

      httplib::Client client(endpoint.origin);
      configure_client(client);

      httplib::Headers headers;
      headers.emplace("Accept", "application/octet-stream");
      for (const auto& kv : options_.headers) {
        headers.emplace(kv.first, kv.second);
      }

      const char* body_ptr = cmd_payload.empty() ? "" : reinterpret_cast<const char*>(cmd_payload.data());
      auto result =
          client.Post(endpoint.command_path, headers, body_ptr, cmd_payload.size(), "application/octet-stream");
      if (!result) {
        last_error = "command transport error (" + httplib::to_string(result.error()) + ")";
        continue;
      }

      if (result->status != 200) {
        last_error = "command request failed (" + std::to_string(result->status) + ")";
        continue;
      }

      if (response_payload) {
        const std::string& body = result->body;
        response_payload->assign(body.begin(), body.end());
      }
      {
        std::lock_guard<std::mutex> lock(state_mu_);
        active_endpoint_index_ = static_cast<int>(i);
      }
      return true;
    }

    if (error) *error = last_error;
    return false;
  }

  DeviceInfo get_info() const override {
    DeviceInfo info;
    info.transport = "http";
    std::lock_guard<std::mutex> lock(state_mu_);
    if (active_endpoint_index_ >= 0 &&
        static_cast<size_t>(active_endpoint_index_) < endpoints_.size()) {
      info.source = endpoints_[static_cast<size_t>(active_endpoint_index_)].base_url;
    } else if (!endpoints_.empty()) {
      info.source = endpoints_[0].base_url;
    } else {
      info.source = "";
    }
    return info;
  }

 private:
  struct Endpoint {
    std::string base_url;
    std::string origin;
    std::string stream_path;
    std::string command_path;
  };

  static std::vector<std::string> default_base_urls() {
    return {
        "http://localhost:8080",
        "http://localhost:8084",
        "http://192.168.7.1:80",
        "http://192.168.7.1:8080",
    };
  }

  static std::string normalize_base_url(const std::string& base_url) {
    if (base_url.size() > 1 && base_url.back() == '/') {
      return base_url.substr(0, base_url.size() - 1);
    }
    return base_url;
  }

  static void append_unique_url(std::vector<std::string>* out, const std::string& value) {
    if (!out) return;
    const std::string normalized = normalize_base_url(value);
    if (normalized.empty()) return;
    for (const auto& existing : *out) {
      if (existing == normalized) return;
    }
    out->push_back(normalized);
  }

  static bool build_endpoints(const MightyWebDeviceOptions& options,
                              std::vector<Endpoint>* endpoints,
                              std::string* error) {
    if (!endpoints) {
      if (error) *error = "internal error: endpoints output is null";
      return false;
    }

    std::vector<std::string> candidates;
    if (!options.base_urls.empty()) {
      for (const auto& base : options.base_urls) append_unique_url(&candidates, base);
    } else if (!options.base_url.empty()) {
      append_unique_url(&candidates, options.base_url);
    } else {
      for (const auto& base : default_base_urls()) append_unique_url(&candidates, base);
    }

    if (candidates.empty()) {
      if (error) *error = "no valid base_url candidates";
      return false;
    }

    const std::string stream_path = options.stream_path.empty() ? "/stream" : options.stream_path;
    const std::string command_path = options.command_path.empty() ? "/command" : options.command_path;
    std::string last_parse_error = "invalid base_url";
    for (const auto& base : candidates) {
      std::string origin;
      std::string base_path_prefix;
      std::string parse_error;
      if (!parse_base_url(base, &origin, &base_path_prefix, &parse_error)) {
        last_parse_error = parse_error;
        continue;
      }

      Endpoint endpoint;
      endpoint.base_url = base;
      endpoint.origin = origin;
      endpoint.stream_path = join_path(base_path_prefix, stream_path);
      endpoint.command_path = join_path(base_path_prefix, command_path);
      endpoints->push_back(endpoint);
    }

    if (endpoints->empty()) {
      if (error) *error = last_parse_error;
      return false;
    }
    return true;
  }

  static bool parse_base_url(const std::string& base_url,
                             std::string* origin,
                             std::string* path_prefix,
                             std::string* error) {
    if (base_url.empty()) {
      if (error) *error = "base_url is required";
      return false;
    }

    const bool has_http = base_url.rfind("http://", 0) == 0;
    const bool has_https = base_url.rfind("https://", 0) == 0;
    if (!has_http && !has_https) {
      if (error) *error = "base_url must start with http:// or https://";
      return false;
    }

    const size_t scheme_end = base_url.find("://") + 3;
    const size_t slash = base_url.find('/', scheme_end);
    if (slash == std::string::npos) {
      *origin = base_url;
      *path_prefix = "";
    } else {
      *origin = base_url.substr(0, slash);
      *path_prefix = base_url.substr(slash);
    }

    if (origin->empty()) {
      if (error) *error = "invalid base_url";
      return false;
    }

    if (!path_prefix->empty() && path_prefix->size() > 1 && path_prefix->back() == '/') {
      path_prefix->pop_back();
    }

    return true;
  }

  static std::string join_path(const std::string& prefix, const std::string& path) {
    const std::string normalized = normalize_path(path);
    if (prefix.empty() || prefix == "/") return normalized;
    if (normalized == "/") return prefix;
    if (prefix.back() == '/') return prefix.substr(0, prefix.size() - 1) + normalized;
    return prefix + normalized;
  }

  static std::string normalize_path(const std::string& path) {
    if (path.empty()) return "/";
    if (path.front() == '/') return path;
    return "/" + path;
  }

  static std::chrono::milliseconds clamp_timeout_ms(int value_ms) {
    if (value_ms <= 0) return std::chrono::milliseconds(1);
    return std::chrono::milliseconds(value_ms);
  }

  void configure_client(httplib::Client& client) const {
    client.set_connection_timeout(clamp_timeout_ms(options_.connect_timeout_ms));
    client.set_read_timeout(clamp_timeout_ms(options_.read_timeout_ms));
    client.set_write_timeout(clamp_timeout_ms(options_.write_timeout_ms));
    client.set_keep_alive(true);
    client.set_follow_location(true);
  }

  bool should_stop() const {
    std::lock_guard<std::mutex> lock(state_mu_);
    return stop_requested_;
  }

  std::vector<size_t> ordered_endpoint_indices() const {
    std::vector<size_t> indices;
    indices.reserve(endpoints_.size());
    int active = -1;
    {
      std::lock_guard<std::mutex> lock(state_mu_);
      active = active_endpoint_index_;
    }
    if (active >= 0 && static_cast<size_t>(active) < endpoints_.size()) {
      indices.push_back(static_cast<size_t>(active));
    }
    for (size_t i = 0; i < endpoints_.size(); ++i) {
      if (static_cast<int>(i) == active) continue;
      indices.push_back(i);
    }
    return indices;
  }

  bool connect_to_endpoint(size_t endpoint_index,
                           BytesCallback on_bytes,
                           std::string* error) {
    if (endpoint_index >= endpoints_.size()) {
      if (error) *error = "invalid endpoint index";
      return false;
    }

    const Endpoint& endpoint = endpoints_[endpoint_index];

    httplib::Client client(endpoint.origin);
    configure_client(client);

    {
      std::lock_guard<std::mutex> lock(state_mu_);
      active_stream_client_ = &client;
    }

    httplib::Headers headers;
    headers.emplace("Accept", "application/octet-stream");
    for (const auto& kv : options_.headers) {
      headers.emplace(kv.first, kv.second);
    }

    std::string stream_error;
    auto result = client.Get(
        endpoint.stream_path,
        headers,
        [&](const httplib::Response& response) {
          if (response.status != 200) {
            stream_error = "stream request failed (" + std::to_string(response.status) + ")";
            return false;
          }
          return true;
        },
        [&](const char* data, size_t len) {
          if (should_stop()) return false;
          if (len > 0) on_bytes(reinterpret_cast<const uint8_t*>(data), len);
          return true;
        });

    const bool stopped_by_user = should_stop();
    {
      std::lock_guard<std::mutex> lock(state_mu_);
      active_stream_client_ = nullptr;
      connected_ = false;
    }

    if (stopped_by_user) return true;
    if (!stream_error.empty()) {
      if (error) *error = stream_error;
      return false;
    }
    if (!result) {
      if (error) *error = "stream transport error (" + httplib::to_string(result.error()) + ")";
      return false;
    }
    if (result->status != 200) {
      if (error) *error = "stream request failed (" + std::to_string(result->status) + ")";
      return false;
    }
    return true;
  }

  MightyWebDeviceOptions options_;
  std::vector<Endpoint> endpoints_;
  int active_endpoint_index_ = -1;

  bool is_valid_ = false;
  std::string init_error_;

  mutable std::mutex state_mu_;
  bool connected_ = false;
  bool stop_requested_ = false;
  httplib::Client* active_stream_client_ = nullptr;
};

}  // namespace sdk
}  // namespace mighty_protocol
