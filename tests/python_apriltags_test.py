import copy
import json
import math
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'python'))
import mighty_protocol as mp
from mighty_sdk import MightyClient, parse_wpilib_field_layout, serialize_apriltag_map_yaml
source = (Path(__file__).parent / 'fixtures/wpilib-2026-rebuilt-welded.json').read_text()
layout = json.loads(source)
tag_map = parse_wpilib_field_layout(source)
assert len(tag_map['tags']) == 32
assert tag_map['field'] == {'length': 16.541, 'width': 8.069}
assert tag_map['tags'][0]['position_m'] == [11.8779798, 7.4247756, 0.889]
assert tag_map['tags'][0]['tag_id'] == 1
assert tag_map['tags'][0]['size_m'] == 0.1651
assert parse_wpilib_field_layout(source.encode()) == tag_map
assert parse_wpilib_field_layout(layout) == tag_map
assert parse_wpilib_field_layout(source, 0.2)['tags'][0]['size_m'] == 0.2
assert layout == json.loads(source), 'do not mutate the source'
yaml = serialize_apriltag_map_yaml(tag_map)
assert yaml.count('orientation: quaternion') == 32
position_line = next(line for line in yaml.splitlines() if 'position_m:' in line)
assert json.loads(position_line.split(':', 1)[1]) == [11.8779798, 7.4247756, 0.889]
for magnitude in (1e-300, 1e300):
    scaled = copy.deepcopy(layout)
    scaled['tags'][0]['pose']['rotation']['quaternion'] = dict(X=0, Y=0, Z=magnitude, W=magnitude)
    q = parse_wpilib_field_layout(scaled)['tags'][0]['orientation_xyzw']
    assert abs(q[2] - math.sqrt(0.5)) < 1e-14
measured = copy.deepcopy(layout)
measured['mighty'] = {'tagFamily': 'tag36h11', 'tagSizeM': 0.12}
assert parse_wpilib_field_layout(measured)['tags'][0]['size_m'] == 0.12
assert parse_wpilib_field_layout(measured, 0.1651)['tags'][0]['size_m'] == 0.1651
for metadata in (None, [], {'tagSizeM': 0}, {'tagSizeM': '0.12'}, {'tagSizeM': None}, {'tagFamily': 'tag16h5'}):
    invalid = dict(measured, mighty=metadata)
    try: parse_wpilib_field_layout(invalid)
    except ValueError: pass
    else: raise AssertionError('invalid metadata accepted')
bad = [{}, {'tags': []}, source + 'garbage', '{broken']
for path, value in [
    (('ID',), -1), (('ID',), 0.5), (('ID',), True), (('ID',), 10**400),
    (('pose','rotation','quaternion'), dict(X=0,Y=0,Z=0,W=0)),
    (('pose','rotation','quaternion'), dict(X=0,Y=0,Z=1)),
    (('pose','translation','x'), float('nan')),
    (('pose','translation','x'), '1'), (('pose','translation','x'), 10**400),
]:
    m = copy.deepcopy(layout); row = m['tags'][0]
    for key in path[:-1]: row = row[key]
    row[path[-1]] = value; bad.append(m)
m = copy.deepcopy(layout); m['tags'].append(m['tags'][0]); bad.append(m)
m = copy.deepcopy(layout); m['field']['width'] = -1; bad.append(m)
for value in bad:
    try: parse_wpilib_field_layout(value)
    except ValueError: pass
    else: raise AssertionError('invalid map accepted')
class Device:
    def __init__(self): self.writes = 0
    def connect(self, _): pass
    def disconnect(self): pass
    def send_command_payload(self, payload):
        cmd = mp.decode_command_payload(payload)
        cfg = mp.decode_config_request_payload(cmd['data'])
        assert cmd['name'] == 'config' and cfg['key'] == 'apriltags'
        assert cfg['op'] == mp.CONFIG_OP['SET']
        assert cfg['value'].decode() == yaml
        self.writes += 1
        data = mp.build_config_response_payload(version=1, op=cfg['op'], success=1,
            has_value=True, key=cfg['key'], value=cfg['value'], message='saved')
        return mp.build_command_response_payload(cmd['req_id'], 0, 'saved', data)
device = Device(); client = MightyClient(device, command_timeout_s=0)
assert client.import_wpilib_field_layout(source)['ok']
for value in bad: assert not client.import_wpilib_field_layout(value)['ok']
assert not client.import_wpilib_field_layout(source, 0)['ok']
assert device.writes == 1, 'invalid layouts must not upload partial maps'
original_yaml = yaml
yaml = serialize_apriltag_map_yaml(parse_wpilib_field_layout(measured))
assert client.import_wpilib_field_layout(measured)['ok']
yaml = original_yaml
if len(sys.argv) > 1: Path(sys.argv[1]).write_text(yaml)
print('python AprilTag importer: official 32-tag layout, full poses, validation, and upload passed')
