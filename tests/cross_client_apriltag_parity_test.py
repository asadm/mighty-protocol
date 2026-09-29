"""Compare SDK payloads numerically, independent of YAML float formatting."""
import json
import math
import os
import subprocess
import sys
import tempfile
from pathlib import Path
here = Path(__file__).resolve().parent
with tempfile.TemporaryDirectory(prefix='mighty-apriltags-') as directory:
    paths = [Path(directory) / (language + '.yaml') for language in ('cpp','js','python')]
    subprocess.run([str(Path(os.environ.get('MIGHTY_TEST_BIN_DIR', str(Path(os.environ.get('TMPDIR', '/tmp')) / 'mighty-protocol-tests'))) / 'cpp_apriltags_test'), str(here/'fixtures/wpilib-2026-rebuilt-welded.json'), str(paths[0])], check=True)
    subprocess.run(['node', str(here/'node_apriltags.test.js'), str(paths[1])], check=True)
    subprocess.run([sys.executable, str(here/'python_apriltags_test.py'), str(paths[2])], check=True)
    def parse(path):
        tags = []
        for line in path.read_text().splitlines():
            text = line.strip()
            if text.startswith('- tag_id:'): tags.append({'id': int(text.split(':')[1])})
            elif ':' in text and tags:
                key, value = text.split(':', 1)
                if key == 'size_m': tags[-1][key] = float(value)
                elif key in ('position_m','orientation_xyzw'): tags[-1][key] = json.loads(value)
                elif key == 'orientation': assert value.strip() == 'quaternion'
        return tags
    payloads = list(map(parse, paths))
    layout = json.loads((here/'fixtures/wpilib-2026-rebuilt-welded.json').read_text())
    for tags in payloads:
        assert len(tags) == 32
        for actual, source in zip(tags, layout['tags']):
            assert actual['id'] == source['ID'] and actual['size_m'] == 0.1651
            assert actual['position_m'] == [source['pose']['translation'][axis] for axis in 'xyz']
    for other in payloads[1:]:
        for expected, actual in zip(payloads[0], other):
            for a, b in zip(expected['orientation_xyzw'], actual['orientation_xyzw']):
                assert math.isclose(a, b, rel_tol=1e-14, abs_tol=1e-14)
print('All three SDKs upload equivalent full-pose field maps')
