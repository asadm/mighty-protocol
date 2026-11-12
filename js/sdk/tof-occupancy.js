import { decodeRawToRgb } from "./image.js";

export function parseTofCalibration(text){
  const block=String(text).match(/^tof_rgb:\s*\n(?:[ \t]+[^\n]*\n|[ \t]*\n)*/m)?.[0]||'';if(!block)throw Error('No depth alignment available');
  const array=key=>{const m=block.match(new RegExp(key+':\\s*\\[([^\\]]+)\\]'));return m?m[1].split(',').map(Number):[];};
  const T=array('T_rgb_tof'),k=array('tof_intrinsics'),size=array('tof_resolution');
  if(T.length!==16||!T.every(Number.isFinite)||k.length!==4||!k.every(Number.isFinite)||k[0]<=0||k[1]<=0||size[0]!==54||size[1]!==42)throw Error('Invalid depth alignment');
  for(let i=0;i<4;i++)if(Math.abs(T[12+i]-(i===3?1:0))>1e-6)throw Error('Invalid transform');
  for(let i=0;i<3;i++)for(let j=0;j<3;j++){let dot=0;for(let n=0;n<3;n++)dot+=T[i*4+n]*T[j*4+n];if(Math.abs(dot-(i===j?1:0))>1e-3)throw Error('Invalid rotation');}
  const det=T[0]*(T[5]*T[10]-T[6]*T[9])-T[1]*(T[4]*T[10]-T[6]*T[8])+T[2]*(T[4]*T[9]-T[5]*T[8]);if(Math.abs(det-1)>1e-3)throw Error('Invalid rotation');
  return {matrix:T,intrinsics:k,resolution:size};
}

// Input depth is already rotated by firmware. Do not rotate it a second time.
export function projectTofOccupancy(image, depth, calibration, camera) {
  if (depth.width !== calibration.resolution[0] || depth.height !== calibration.resolution[1]) return null;
  const source = decodeRawToRgb(image);
  if (!source || camera.cameraModel !== "double_sphere" || camera.distortionModel !== "none")
    throw new Error("ToF occupancy requires the calibrated double-sphere RGB camera");
  const T = calibration.matrix, [fx, fy, cx, cy] = calibration.intrinsics;
  const [xi, alpha] = camera.projectionParameters, k = camera.intrinsics;
  const sx = source.width / camera.resolution.width, sy = source.height / camera.resolution.height;
  const xyz = [], rgb = [];
  const scale = Number(depth.depthScaleM);
  if (!Number.isFinite(scale) || scale <= 0) return null;
  for (let y=0; y<depth.height; y++) for(let x=0; x<depth.width; x++) {
    const v=depth.depthMm[y*depth.width+x];
    if (!v || v===depth.invalidValue) continue;
    const z=v*scale, X=(x-cx)*z/fx, Y=(y-cy)*z/fy;
    if (!Number.isFinite(z) || z<=0 || z>10) continue;
    const a=T[0]*X+T[1]*Y+T[2]*z+T[3], b=T[4]*X+T[5]*Y+T[6]*z+T[7], c=T[8]*X+T[9]*Y+T[10]*z+T[11];
    if(c<=0) continue;
    const d=Math.hypot(a,b,c), q=xi*d+c, den=alpha*Math.hypot(a,b,q)+(1-alpha)*q;
    if(den<=0) continue;
    const u=Math.round((k.fx*a/den+k.cx)*sx), w=Math.round((k.fy*b/den+k.cy)*sy);
    if(u<0||w<0||u>=source.width||w>=source.height) continue;
    const i=(w*source.width+u)*4;
    if (!source.rgba[i+3]) continue;
    xyz.push(a,b,c); rgb.push(source.rgba[i],source.rgba[i+1],source.rgba[i+2]);
  }
  return {xyz:new Float32Array(xyz),rgb:new Uint8Array(rgb),origin:new Float32Array([T[3],T[7],T[11]])};
}
