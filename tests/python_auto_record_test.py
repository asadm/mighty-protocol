import json
import pathlib
import sys
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'python'))
from mighty_sdk import MightyClient

client = object.__new__(MightyClient)
calls = []
response = {'ok': True, 'data': json.dumps({'enabled': True, 'active': False}).encode()}
def command(name, data):
    calls.append((name, data))
    return response
client.command = command
assert client.set_auto_record_with_vio(True)['value']['enabled']
client.set_auto_record_with_vio(False)
client.get_auto_record_with_vio()
assert calls == [('auto_record_vio', b'on'), ('auto_record_vio', b'off'), ('auto_record_vio', b'status')]
try:
    client.set_auto_record_with_vio('false')
    raise AssertionError('accepted nonboolean')
except TypeError:
    pass
for data in [b'null', b'{}', b'broken']:
    response = {'ok': True, 'data': data}
    assert not client.get_auto_record_with_vio()['ok']
response = {'ok': False, 'message': 'unknown command'}
assert client.get_auto_record_with_vio() == response
print('automatic recording Python SDK passed')
