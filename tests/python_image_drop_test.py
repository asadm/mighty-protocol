import sys, struct
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'python'))
import mighty_protocol as mp
from mighty_sdk.image import select_primary_image
payload=struct.pack('>QBQB',123,1,19,7)+b'preview'
d=mp.decode_image_drop_payload(payload)
assert d==dict(timestamp_ns=123,dropped=True,dropped_count=19,channel='preview',data=b'')
assert select_primary_image(dict(d,kind='jpg')) is None
for bad in (payload[:-1],payload+b'x',b'\0'*18):
 try: mp.decode_image_drop_payload(bad)
 except ValueError: pass
 else: raise AssertionError('accepted malformed marker')
print('PASS Python drop metadata and empty-image bypass')
