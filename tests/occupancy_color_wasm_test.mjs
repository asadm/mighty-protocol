// Usage: node tests/occupancy_color_wasm_test.mjs /path/to/mighty_algorithms.js
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { pathToFileURL } from 'node:url';
import { resolve, dirname, join } from 'node:path';
import { MightyOccupancyGrid } from '../js/sdk/occupancy-wasm.js';

const modulePath = resolve(process.argv[2] || 'lib/algorithms/wasm/lib/mighty_algorithms.js');
const { default: createModule } = await import(pathToFileURL(modulePath));
const m = await createModule({ wasmBinary: readFileSync(join(dirname(modulePath), 'mighty_algorithms.wasm')) });
const noop = () => () => {};
const client = { onImage: noop, onPose: noop, onVioState: noop, onReset: noop };
const grid = new MightyOccupancyGrid(client, m, { autoProcess: false, calibration: {
  cameras: [{ id: 'cam0', valid: true, width: 640, height: 400,
    resolution: { width: 640, height: 400 }, intrinsics: { fx: 400, fy: 400, cx: 320, cy: 200 },
    cameraModel: 'pinhole' }],
} });
const native = grid.native;
const l = native.layout;
assert.equal(l.cellRgbOffset, 20);
assert.equal(l.cellSize, 24);
const ptr = m._malloc(l.updateSize);
const cells = m._malloc(l.cellSize * 3);
m.HEAPU8.fill(0, ptr, ptr + l.updateSize);
m.HEAPU8.fill(0, cells, cells + l.cellSize * 3);
m.setValue(ptr + l.updateChangesOffset, cells, '*');
m.setValue(ptr + l.updateChangeCountOffset, 3, 'i32');
const colors = [255, 0, 0, 0, 0, 255, 80, 80, 80];
for (let i = 0; i < 3; ++i) {
  const cell = cells + i * l.cellSize;
  m.setValue(cell + l.cellIndexXOffset, i - 1, 'i32');
  m.setValue(cell + l.cellStateOffset, 2, 'i32');
  m.setValue(cell + l.cellIntensityOffset, 80, 'i8');
  m.HEAPU8.set(colors.slice(i * 3, i * 3 + 3), cell + l.cellRgbOffset);
}
const update = native._readUpdate(ptr);
assert.deepEqual([...update.rgb], colors);
assert.deepEqual([...update.indices], [-1, 0, 0, 0, 0, 0, 1, 0, 0]);
const transferred = structuredClone(update, { transfer: [update.rgb.buffer] });
assert.deepEqual([...transferred.rgb], colors);
assert.equal(update.rgb.byteLength, 0);
// The decoder also accepts the old layout without an RGB extension.
native.layout.cellRgbOffset = null;
assert.deepEqual([...native._readUpdate(ptr).rgb], Array(9).fill(80));
native.layout.cellRgbOffset = 20;
m._free(cells); m._free(ptr);
assert.equal(native.clear().rgb.length, 0);
grid.close();
console.log('WASM RGB layout, decoder, transfer and grayscale fallback passed');
