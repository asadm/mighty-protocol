#include <cassert>
#include <fstream>
#include <iostream>
#include <iterator>
#include "../cpp/mighty_sdk.h"
using namespace mighty_protocol;
using namespace mighty_protocol::sdk;
class Device : public MightyDeviceIO {
 public:
  std::string expected;
  int writes = 0;
  DeviceInfo get_info() const override { return {"mock", "mock://device"}; }
  bool connect(BytesCallback, std::string*) override { return true; }
  void disconnect() override {}
  bool send_command_payload(const std::vector<uint8_t>& payload,
                            std::vector<uint8_t>* response, std::string*) override {
    CommandRequest cmd; assert(decode_command_payload(payload, cmd));
    ConfigRequest cfg; assert(decode_config_request_payload(cmd.data, cfg));
    assert(cmd.name == "config" && cfg.key == "apriltags");
    assert(cfg.op == static_cast<uint8_t>(ConfigOp::kSet));
    assert(std::string(cfg.value.begin(), cfg.value.end()) == expected); ++writes;
    ConfigResponse result; result.version = 1; result.op = cfg.op; result.success = 1;
    result.has_value = true; result.key = cfg.key; result.value = cfg.value;
    CommandResponse cres; cres.req_id = cmd.req_id; cres.status = 0;
    cres.data = build_config_response_payload(result);
    *response = build_command_response_payload(cres); return true;
  }
};
int main(int argc, char** argv) {
  assert(argc >= 2);
  std::ifstream file(argv[1]); assert(file.good());
  std::string source((std::istreambuf_iterator<char>(file)), {});
  AprilTagMap map; std::string error;
  assert(parse_wpilib_field_layout_json(source, &map, &error));
  assert(error.empty() && map.tags.size() == 32);
  assert(map.field_length_m == 16.541 && map.field_width_m == 8.069);
  assert(map.tags[0].tag_id == 1 && map.tags[0].size_m == 0.1651);
  assert((map.tags[0].position_m == std::array<double,3>{11.8779798, 7.4247756, 0.889}));
  auto yaml = serialize_apriltag_map_yaml(map);
  assert(yaml.find("orientation: quaternion") != std::string::npos);
  AprilTagMap custom;
  assert(parse_wpilib_field_layout_json(source, &custom, &error, 0.2));
  assert(custom.tags[0].size_m == 0.2);
  const std::string one = R"({"tags":[{"ID":7,"pose":{"translation":{"x":1,"y":2,"z":3},"rotation":{"quaternion":{"X":0,"Y":0,"Z":0,"W":1}}}}]})";
  assert(parse_wpilib_field_layout_json(one, &custom, &error));
  assert(serialize_apriltag_map_yaml(custom).find("orientation_xyzw: [-0.5, 0.5, -0.5, 0.5]") != std::string::npos);
  const std::string measured = one.substr(0, one.size()-1) + R"(,"mighty":{"tagFamily":"tag36h11","tagSizeM":0.12}})";
  assert(parse_wpilib_field_layout_json(measured, &custom, &error));
  assert(custom.tags[0].size_m == 0.12);
  assert(parse_wpilib_field_layout_json(measured, &custom, &error, 0.1651));
  assert(custom.tags[0].size_m == 0.1651);
  for (const std::string metadata : {"null", "[]", "{\"tagSizeM\":0}", "{\"tagSizeM\":\"0.12\"}", "{\"tagSizeM\":null}", "{\"tagFamily\":\"tag16h5\"}"}) {
    assert(!parse_wpilib_field_layout_json(one.substr(0, one.size()-1) + ",\"mighty\":" + metadata + "}", &custom, &error));
  }
  std::vector<std::string> bad{"{}", "{broken", "{\"tags\":[]}", source + "garbage"};
  for (const auto& replacement : {std::string("-1"), std::string("0.5"), std::string("true")}) {
    auto input = one; auto p = input.find("\"ID\":7"); input.replace(p, 6, "\"ID\":" + replacement); bad.push_back(input);
  }
  auto zero = one; auto p = zero.find("\"W\":1"); zero.replace(p, 5, "\"W\":0"); bad.push_back(zero);
  auto duplicate = map; duplicate.tags.push_back(map.tags[0]);
  try { serialize_apriltag_map_yaml(duplicate); assert(false); } catch(const std::invalid_argument&) {}
  for (const auto& input : bad) {
    assert(!parse_wpilib_field_layout_json(input, &map, &error));
    assert(!error.empty() && map.tags.size() == 32);
  }
  assert(!parse_wpilib_field_layout_json(source, &map, &error, 0));
  for (double magnitude : {1e-300, 1e300}) {
    auto q = apriltag_detail::normalize({0, 0, magnitude, magnitude});
    assert(std::abs(q[2] - std::sqrt(0.5)) < 1e-14);
  }
  auto device = std::make_shared<Device>(); device->expected = yaml;
  MightyClient client(device);
  assert(client.import_wpilib_field_layout(source).ok);
  for (const auto& input : bad) assert(!client.import_wpilib_field_layout(input).ok);
  assert(device->writes == 1);
  assert(parse_wpilib_field_layout_json(measured, &custom, &error));
  device->expected = serialize_apriltag_map_yaml(custom);
  assert(client.import_wpilib_field_layout(measured).ok);
  if (argc >= 3) { std::ofstream output(argv[2]); output << yaml; }
  std::cout << "C++ AprilTag importer: official 32-tag layout, full poses, validation, and upload passed\n";
}
