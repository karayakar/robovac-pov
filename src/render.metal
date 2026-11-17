// render.metal — path tracer for the robot-vacuum film. One compute kernel renders, one bakes the bounce-light grid.
// Geometry is triangles in hardware acceleration structures; every material is a procedural function below.
#include <metal_stdlib>
#include <metal_raytracing>
using namespace metal;
using namespace raytracing;

#define PI 3.14159265358979f
#define MASK_WORLD 1u
#define MASK_ROBOT 2u
#define EPS 2.5e-4f

enum { M_FLOOR = 0, M_WOOD, M_PAINT, M_TRIM, M_FABRIC, M_RUG, M_PLASTIC, M_METAL, M_CERAMIC, M_LEAF, M_ROBOT, M_EMIT, M_MATTE,
       M_FACADE, M_FOLIAGE, M_RUBBER, M_CONCRETE, M_CURTAIN, M_BOOK, M_ART, M_SKIN, M_SOCKET };

struct Uni {
  float4 camP0, camP1;
  float4 camR0, camU0, camF0, camR1, camU1, camF1;
  float4 lens;      // x: stereographic k = tan(hfov/4), y: aperture radius (m), z: focus distance (m), w: height/width
  float4 sunDir;    // xyz: towards the sun, w: cos(angular radius)
  float4 sunCol;    // irradiance on a surface facing the sun
  float4 roomLo, roomHi; // interior box; roomHi.z is the portal plane (outer face of the window wall)
  float4 gridLo, gridInv;
  float4 portal[4]; // x0, x1, y0, y1
  float4 misc;      // x: time (s), y: indirect clamp, z: fog density, w: sum of portal area * |n.s|
  float4 misc2;     // x: total portal area, y: sky scale, z: shell present (1/0), w: unused
  uint4 counts;     // x: seed, y: number of portals, z: samples per pixel this pass, w: first triangle of the camera-space shell
  uint4 dims;       // x: width, y: height, z: debug mode, w: unused
  uint4 grid;       // x,y,z: voxel counts, w: rays per voxel per pass (bake)
};
struct Inst { float4 r0, r1, r2; uint4 meta; }; // rows of object->world, meta.x = first triangle
struct TriA { uint mat; uint col; };

struct Ctx {
  instance_acceleration_structure tlas;
  primitive_acceleration_structure shell;
  const device uint* idx;
  const device packed_float3* pos;
  const device packed_float3* nrm;
  const device packed_float3* tex;
  const device TriA* tri;
  const device Inst* inst;
  texture3d<float> g0, g1, g2, g3, g4, g5;
};

// ---------------------------------------------------------------- random and noise
static uint pcg(uint v) { uint s = v * 747796405u + 2891336453u; uint w = ((s >> ((s >> 28u) + 4u)) ^ s) * 277803737u; return (w >> 22u) ^ w; }
static float rnd(thread uint& s) { s = pcg(s); return float(s >> 8) * (1.0f / 16777216.0f); }
static float hashi(int3 c) { return float(pcg(uint(c.x) * 73856093u ^ uint(c.y) * 19349663u ^ uint(c.z) * 83492791u) >> 8) * (1.0f / 16777216.0f); }
static float vnoise(float3 p) {
  float3 fl = floor(p); int3 i = int3(fl); float3 f = p - fl; f = f * f * (3.0f - 2.0f * f);
  float a = mix(hashi(i), hashi(i + int3(1, 0, 0)), f.x), b = mix(hashi(i + int3(0, 1, 0)), hashi(i + int3(1, 1, 0)), f.x);
  float c = mix(hashi(i + int3(0, 0, 1)), hashi(i + int3(1, 0, 1)), f.x), d = mix(hashi(i + int3(0, 1, 1)), hashi(i + int3(1, 1, 1)), f.x);
  return mix(mix(a, b, f.y), mix(c, d, f.y), f.z);
}
static float fbm(float3 p, int oct) { float a = 0.5f, s = 0; for (int i = 0; i < oct; i++) { s += a * vnoise(p); p = p * 2.03f + float3(17.1f, 3.3f, 9.7f); a *= 0.5f; } return s; }
static float lum(float3 c) { return dot(c, float3(0.2126f, 0.7152f, 0.0722f)); }
static float3 srgb(uint c) { float3 v = float3(float(c & 255u), float((c >> 8) & 255u), float((c >> 16) & 255u)) * (1.0f / 255.0f); return pow(v, float3(2.2f)); }
static void basis(float3 n, thread float3& t, thread float3& b) {
  float s = n.z >= 0 ? 1.0f : -1.0f; float a = -1.0f / (s + n.z); float c = n.x * n.y * a;
  t = float3(1.0f + s * n.x * n.x * a, s * c, -s * n.x); b = float3(c, s + n.y * n.y * a, -n.y);
}

// ---------------------------------------------------------------- sky
static float3 sky(float3 d, constant Uni& U) {
  float3 s = U.sunDir.xyz; float cs = dot(d, s); float h = max(d.y, 0.0f);
  float3 c = mix(float3(0.62f, 0.72f, 0.86f), float3(0.13f, 0.27f, 0.66f), pow(h, 0.42f));
  c += float3(1.0f, 0.72f, 0.45f) * (0.55f * pow(max(cs, 0.0f), 28.0f) + 0.16f * pow(max(cs, 0.0f), 5.0f));
  if (d.y > 0.01f) {
    float2 uv = d.xz / (d.y + 0.12f) * 0.9f;
    float cl = fbm(float3(uv.x, uv.y, 3.7f), 5);
    cl = smoothstep(0.50f, 0.74f, cl) * smoothstep(0.01f, 0.12f, d.y);
    float3 cc = float3(1.0f, 0.96f, 0.90f) * (0.85f + 0.9f * pow(max(cs, 0.0f), 3.0f));
    c = mix(c, cc, cl * 0.85f);
  } else {
    c = mix(c, float3(0.20f, 0.21f, 0.20f), smoothstep(0.0f, -0.08f, d.y)); // far ground haze
  }
  return c * U.misc2.y;
}
static float3 skyAmbientE(float3 n, constant Uni& U) { // rough irradiance from the open sky on an outdoor surface
  return float3(0.36f, 0.50f, 0.78f) * U.misc2.y * PI * (0.62f + 0.38f * n.y);
}

// ---------------------------------------------------------------- tracing
struct Hit { bool ok; bool shell; float t; uint prim; uint inst; float2 bc; };
static Hit traceWorld(thread Ctx& C, float3 o, float3 d, float tmin, float tmax, uint mask) {
  intersector<triangle_data, instancing> isect;
  isect.assume_geometry_type(geometry_type::triangle);
  isect.force_opacity(forced_opacity::opaque);
  ray r; r.origin = o; r.direction = d; r.min_distance = tmin; r.max_distance = tmax;
  intersection_result<triangle_data, instancing> h = isect.intersect(r, C.tlas, mask);
  Hit o2; o2.ok = h.type != intersection_type::none; o2.shell = false; o2.t = h.distance; o2.prim = h.primitive_id; o2.inst = h.instance_id; o2.bc = h.triangle_barycentric_coord;
  return o2;
}
static bool occluded(thread Ctx& C, float3 o, float3 d, float tmax, uint mask) {
  intersector<triangle_data, instancing> isect;
  isect.assume_geometry_type(geometry_type::triangle);
  isect.force_opacity(forced_opacity::opaque);
  isect.accept_any_intersection(true);
  ray r; r.origin = o; r.direction = d; r.min_distance = 0.0f; r.max_distance = tmax;
  intersection_result<triangle_data, instancing> h = isect.intersect(r, C.tlas, mask);
  return h.type != intersection_type::none;
}

// ---------------------------------------------------------------- surfaces
struct Surf {
  float3 p, n, ng; float3 alb, F0, trans, emis; float rough; uint mat; bool back; bool metal;
};
static bool insideRoom(float3 p, constant Uni& U) { return all(p > U.roomLo.xyz) && all(p < U.roomHi.xyz); }

static Surf getSurf(thread Ctx& C, Hit h, float3 o, float3 d, bool lite, constant Uni& U, float3 cR, float3 cU, float3 cF) {
  Surf s; s.p = o + d * h.t;
  uint gp = (h.shell ? U.counts.w : C.inst[h.inst].meta.x) + h.prim;
  uint i0 = C.idx[gp * 3], i1 = C.idx[gp * 3 + 1], i2 = C.idx[gp * 3 + 2];
  float w1 = h.bc.x, w2 = h.bc.y, w0 = 1.0f - w1 - w2;
  float3 P0 = C.pos[i0], P1 = C.pos[i1], P2 = C.pos[i2];
  float3 ngo = cross(P1 - P0, P2 - P0);
  float3 no = float3(C.nrm[i0]) * w0 + float3(C.nrm[i1]) * w1 + float3(C.nrm[i2]) * w2;
  float3 tx = float3(C.tex[i0]) * w0 + float3(C.tex[i1]) * w1 + float3(C.tex[i2]) * w2;
  float3 ng, n;
  if (h.shell) { ng = ngo.x * cR + ngo.y * cU + ngo.z * cF; n = no.x * cR + no.y * cU + no.z * cF; }
  else if (h.inst == 0) { ng = ngo; n = no; }
  else { Inst I = C.inst[h.inst]; ng = float3(dot(I.r0.xyz, ngo), dot(I.r1.xyz, ngo), dot(I.r2.xyz, ngo)); n = float3(dot(I.r0.xyz, no), dot(I.r1.xyz, no), dot(I.r2.xyz, no)); }
  ng = normalize(ng); n = normalize(n);
  s.back = dot(ng, d) > 0.0f;
  if (s.back) ng = -ng;
  if (dot(n, ng) < 0.0f) n = -n;
  s.ng = ng;
  TriA ta = C.tri[gp]; uint mat = ta.mat; s.mat = mat;
  float3 col = srgb(ta.col); float pa = float(ta.col >> 24) * (1.0f / 255.0f);
  s.alb = col; s.F0 = float3(0.04f); s.trans = float3(0); s.emis = float3(0); s.rough = 0.6f; s.metal = false;
  float dist = h.t;
  switch (mat) {
    case M_FLOOR: {
      // oak: long soft streaks of varying width (no regular stripes), fine pores, slow colour drift along each plank
      float w = tx.z + 0.012f * sin(tx.x * 1.7f + pa * 40.0f) + 0.010f * (vnoise(float3(tx.x * 2.5f, pa * 50.0f, 1.0f)) - 0.5f);
      float rings = fbm(float3(tx.x * 0.9f, w * 95.0f, pa * 13.0f), 4);
      rings = smoothstep(0.25f, 0.75f, rings);
      float fine = lite ? 0.5f : vnoise(float3(tx.x * 45.0f, w * 1300.0f, pa * 7.0f));
      float drift = vnoise(float3(tx.x * 1.1f, pa * 31.0f, 4.0f));
      float3 base = mix(float3(0.335f, 0.215f, 0.118f), float3(0.455f, 0.315f, 0.180f), pa * 0.7f + drift * 0.3f);
      s.alb = base * (0.86f + 0.20f * rings) * (0.93f + 0.14f * fine);
      float wear = lite ? 0.5f : fbm(float3(s.p.x * 2.1f, 0.3f, s.p.z * 2.1f), 3);
      s.rough = 0.20f + 0.12f * wear + 0.05f * fine;
      if (!lite) { float g2 = vnoise(float3(tx.x * 55.0f, (w + 0.0006f) * 1100.0f, pa * 7.0f)); n = normalize(n + float3(0, 0, 1) * (g2 - fine) * 0.05f); }
    } break;
    case M_WOOD: {
      float along = pa < 0.5f ? tx.x : tx.y; float3 q = pa < 0.5f ? float3(tx.y, tx.z, 0) : float3(tx.x, tx.z, 0);
      float w = q.x * 1.0f + q.y * 0.6f + 0.006f * sin(along * 7.0f);
      float rings = 0.5f + 0.5f * sin(w * 320.0f + 4.0f * vnoise(float3(along * 3.0f, w * 30.0f, 2.0f)));
      float fine = lite ? 0.5f : vnoise(float3(along * 60.0f, q.x * 900.0f, q.y * 900.0f));
      s.alb = col * (0.78f + 0.30f * rings) * (0.9f + 0.2f * fine);
      s.rough = 0.30f + 0.1f * fine;
    } break;
    case M_PAINT: {
      s.rough = 0.85f; s.F0 = float3(0.03f);
      if (!lite && dist < 3.0f) { float e = 0.0015f; float3 q = s.p * 420.0f; float n0 = vnoise(q); n = normalize(n + 0.035f * float3(vnoise(q + float3(e * 420.0f, 0, 0)) - n0, vnoise(q + float3(0, e * 420.0f, 0)) - n0, vnoise(q + float3(0, 0, e * 420.0f)) - n0) * 6.0f); }
    } break;
    case M_TRIM: { s.rough = 0.32f; } break;
    case M_FABRIC: {
      float3 an = abs(n); float2 uv = an.y > an.x && an.y > an.z ? tx.xz : (an.x > an.z ? tx.yz : tx.xy);
      float fade = lite ? 0.0f : saturate(1.6f - dist * 0.55f);
      float wv = sin(uv.x * 1500.0f) * sin(uv.y * 1500.0f);
      float mot = fbm(float3(uv * 14.0f, pa * 9.0f), 3);
      s.alb = col * (0.84f + 0.2f * mot) * (1.0f + 0.16f * wv * fade);
      s.rough = 0.95f; s.F0 = float3(0.02f);
      if (!lite) { float fz = pow(1.0f - saturate(dot(n, -d)), 3.0f); s.alb *= 1.0f + 0.45f * fz; }
    } break;
    case M_RUG: {
      float2 q = tx.xz / 0.46f; q += 0.06f * float2(vnoise(float3(tx.xz * 3.0f, 1.0f)), vnoise(float3(tx.xz * 3.0f, 5.0f)));
      float d1 = abs(fract(q.x + q.y) - 0.5f), d2 = abs(fract(q.x - q.y) - 0.5f);
      float line = smoothstep(0.035f, 0.022f, min(d1, d2));
      float3 cream = float3(0.60f, 0.54f, 0.44f), dark = float3(0.105f, 0.092f, 0.082f);
      float v = 0.88f + 0.22f * pa;
      s.alb = mix(cream, dark, line) * v * mix(0.55f, 1.0f, saturate(tx.y));
      s.rough = 1.0f; s.F0 = float3(0.015f);
      if (tx.y > 0.05f) n = normalize(ng * 0.45f + float3(0, 1, 0)); // pile: both faces of a blade shade as "mostly up"
    } break;
    case M_PLASTIC: { s.rough = 0.06f + 0.6f * pa; s.F0 = float3(0.045f); } break;
    case M_METAL: { s.metal = true; s.F0 = col; s.alb = float3(0); s.rough = 0.08f + 0.6f * pa; } break;
    case M_CERAMIC: {
      if (pa > 0.5f) { float nz = lite ? 0.5f : fbm(tx * 260.0f, 3); s.alb = col * (0.8f + 0.4f * nz); s.rough = 0.9f; s.F0 = float3(0.02f); }
      else { s.rough = 0.09f; s.F0 = float3(0.05f); }
    } break;
    case M_LEAF: {
      float u = tx.x, v = tx.y;
      float mid = smoothstep(0.045f, 0.0f, abs(v));
      float side = smoothstep(0.75f, 1.0f, abs(sin((u * 9.0f - abs(v) * 2.2f) * PI))) * 0.5f;
      float nz = fbm(float3(u * 9.0f, v * 5.0f, tx.z * 31.0f), 3);
      s.alb = col * (0.75f + 0.5f * nz); s.alb = mix(s.alb, float3(0.42f, 0.50f, 0.20f), max(mid, side * 0.35f));
      s.trans = s.alb * float3(0.75f, 1.0f, 0.3f) * 0.55f;
      s.rough = 0.22f + 0.1f * nz; s.F0 = float3(0.045f);
    } break;
    case M_ROBOT: {
      float r = length(tx.xz);
      s.alb = col; s.rough = 0.07f; s.F0 = float3(0.045f);
      if (!lite) {
        float sm = fbm(float3(tx.x * 14.0f, 2.0f, tx.z * 14.0f), 4);
        s.rough += 0.16f * smoothstep(0.42f, 0.75f, sm);
        float du = vnoise(float3(tx.x * 3600.0f, 7.0f, tx.z * 3600.0f)); float du2 = vnoise(float3(tx.x * 900.0f, 3.0f, tx.z * 900.0f));
        float dust = smoothstep(0.925f, 0.975f, du) * smoothstep(0.35f, 0.7f, du2 + 0.25f * sm);
        s.alb = mix(s.alb, float3(0.30f, 0.285f, 0.26f), dust * 0.5f); s.rough = mix(s.rough, 0.9f, dust);
        float scr = vnoise(float3(r * 5200.0f, atan2(tx.z, tx.x) * 3.0f, 1.0f)); s.rough += 0.05f * scr;
      }
      if (pa > 0.75f) { s.rough = 0.42f; }
    } break;
    case M_EMIT: { s.emis = col * pa * 40.0f; s.alb = float3(0.02f); } break;
    case M_MATTE: { s.rough = 1.0f; s.F0 = float3(0.02f); if (!lite) s.alb *= 0.85f + 0.3f * vnoise(tx * 900.0f); } break;
    case M_FACADE: {
      float2 q = float2(tx.x / 3.1f, tx.y / 2.9f); float2 f = fract(q); float2 id = floor(q);
      float win = step(0.22f, f.x) * step(f.x, 0.78f) * step(0.25f, f.y) * step(f.y, 0.80f);
      float hsh = hashi(int3(int(id.x), int(id.y), 7));
      s.alb = mix(col * (0.9f + 0.2f * vnoise(float3(tx.xy * 0.7f, 1.0f))), float3(0.05f, 0.07f, 0.10f) + 0.25f * step(0.8f, hsh), win);
      s.rough = mix(0.9f, 0.15f, win);
    } break;
    case M_FOLIAGE: { float nz = fbm(tx * 5.0f, 4), n2 = fbm(tx * 23.0f, 3); s.alb = col * (0.25f + 1.1f * nz) * (0.45f + 1.1f * n2); s.rough = 0.8f; s.trans = s.alb * 0.25f;
      if (!lite) n = normalize(n + 0.9f * (float3(fbm(tx * 23.0f + 7.1f, 2), fbm(tx * 23.0f + 13.3f, 2), fbm(tx * 23.0f + 29.7f, 2)) - 0.5f)); } break;
    case M_RUBBER: { s.rough = 0.55f; s.F0 = float3(0.035f); } break;
    case M_CONCRETE: { float nz = lite ? 0.5f : fbm(tx * 30.0f, 4); s.alb = col * (0.75f + 0.5f * nz); s.rough = 0.9f; } break;
    case M_CURTAIN: {
      float wv = lite ? 0.0f : sin(tx.x * 2600.0f) * sin(tx.y * 2600.0f) * saturate(1.5f - dist * 0.6f);
      s.alb = col * (0.92f + 0.08f * wv); s.trans = col * 0.42f; s.rough = 0.95f; s.F0 = float3(0.02f);
    } break;
    case M_BOOK: {
      float band = step(0.035f, tx.y) * step(tx.y, 0.075f) + step(-0.085f, tx.y) * step(tx.y, -0.075f);
      float3 lab = pa > 0.5f ? float3(0.75f, 0.70f, 0.60f) : float3(0.03f);
      s.alb = mix(col, lab, band * 0.9f);
      if (n.y > 0.7f) s.alb = float3(0.72f, 0.68f, 0.58f) * (0.85f + 0.15f * sin(tx.x * 4000.0f));
      s.rough = 0.55f;
    } break;
    case M_ART: {
      float2 q = tx.xy; float sd = tx.z;
      float3 bg = float3(0.80f, 0.74f, 0.64f); float3 c = bg;
      for (int k = 0; k < 4; k++) {
        float fk = float(k) + sd * 10.0f; float2 cc = float2(0.25f + 0.5f * fract(sin(fk * 12.9f) * 43758.5f), 0.25f + 0.5f * fract(sin(fk * 78.2f) * 12543.1f));
        float rr = 0.12f + 0.16f * fract(sin(fk * 3.7f) * 9631.7f);
        float3 pc = k == 0 ? float3(0.62f, 0.25f, 0.13f) : (k == 1 ? float3(0.10f, 0.20f, 0.26f) : (k == 2 ? float3(0.75f, 0.52f, 0.20f) : float3(0.30f, 0.38f, 0.30f)));
        float dd = k % 2 == 0 ? length(q - cc) - rr : max(abs(q.x - cc.x) - rr * 0.7f, abs(q.y - cc.y) - rr * 1.3f);
        c = mix(c, pc, smoothstep(0.004f, 0.0f, dd));
      }
      s.alb = c; s.rough = 0.7f;
    } break;
    case M_SKIN: {
      s.rough = 0.36f; s.F0 = float3(0.045f);
      if (!lite) {
        float3 q = tx * 520.0f; float e = 0.35f; float n0 = fbm(q, 2);
        float3 g = float3(fbm(q + float3(e, 0, 0), 2) - n0, fbm(q + float3(0, e, 0), 2) - n0, fbm(q + float3(0, 0, e), 2) - n0);
        n = normalize(n + g * 0.45f * saturate(1.4f - dist * 1.2f));
        s.alb *= 0.90f + 0.2f * n0;
      }
    } break;
    default: break;
  }
  if (dot(n, ng) < 0.05f) n = normalize(n + ng * 0.3f);
  if (dot(n, -d) < 0.02f) n = normalize(n - d * (0.04f - dot(n, -d))); // keep the shading normal facing the viewer
  s.n = n;
  return s;
}

// ---------------------------------------------------------------- BSDF helpers
static float3 fresnel(float3 F0, float c) { float m = 1.0f - saturate(c); float m2 = m * m; return F0 + (1.0f - F0) * (m2 * m2 * m); }
static float G1(float nx, float a) { float a2 = a * a; return 2.0f * nx / (nx + sqrt(a2 + (1.0f - a2) * nx * nx)); }
static float3 specEval(Surf s, float3 v, float3 l) {
  float a = max(s.rough * s.rough, 0.003f); float3 hv = normalize(v + l);
  float nh = saturate(dot(s.n, hv)), nv = max(dot(s.n, v), 1e-4f), nl = max(dot(s.n, l), 1e-4f);
  float a2 = a * a; float dd = nh * nh * (a2 - 1.0f) + 1.0f; float D = a2 / (PI * dd * dd);
  return fresnel(s.F0, dot(v, hv)) * (D * G1(nv, a) * G1(nl, a) / (4.0f * nv * nl));
}
static float3 sampleVNDF(float3 Ve, float a, float2 u) {
  float3 Vh = normalize(float3(a * Ve.x, a * Ve.y, Ve.z));
  float lensq = Vh.x * Vh.x + Vh.y * Vh.y;
  float3 T1 = lensq > 1e-12f ? float3(-Vh.y, Vh.x, 0) / sqrt(lensq) : float3(1, 0, 0);
  float3 T2 = cross(Vh, T1);
  float r = sqrt(u.x), phi = 2.0f * PI * u.y; float t1 = r * cos(phi), t2 = r * sin(phi);
  float sv = 0.5f * (1.0f + Vh.z); t2 = (1.0f - sv) * sqrt(max(0.0f, 1.0f - t1 * t1)) + sv * t2;
  float3 Nh = t1 * T1 + t2 * T2 + sqrt(max(0.0f, 1.0f - t1 * t1 - t2 * t2)) * Vh;
  return normalize(float3(a * Nh.x, a * Nh.y, max(1e-6f, Nh.z)));
}
static float3 coneDir(float3 s, float cosMax, float2 u) {
  float ct = 1.0f - u.x * (1.0f - cosMax), st = sqrt(max(0.0f, 1.0f - ct * ct)), ph = 2.0f * PI * u.y;
  float3 t, b; basis(s, t, b); return normalize(t * (st * cos(ph)) + b * (st * sin(ph)) + s * ct);
}

// bounce-light grid: six directional irradiance volumes (an ambient cube per voxel)
static float3 gridE(thread Ctx& C, float3 p, float3 n, constant Uni& U) {
  constexpr sampler smp(filter::linear, address::clamp_to_edge);
  p = clamp(p, U.roomLo.xyz + 0.06f, U.roomHi.xyz - 0.06f); // never read the voxels outside the room (they see open sky)
  float3 uvw = (p - U.gridLo.xyz) * U.gridInv.xyz; float3 n2 = n * n;
  float3 ex = n.x >= 0 ? C.g0.sample(smp, uvw).rgb : C.g1.sample(smp, uvw).rgb;
  float3 ey = n.y >= 0 ? C.g2.sample(smp, uvw).rgb : C.g3.sample(smp, uvw).rgb;
  float3 ez = n.z >= 0 ? C.g4.sample(smp, uvw).rgb : C.g5.sample(smp, uvw).rgb;
  return n2.x * ex + n2.y * ey + n2.z * ez;
}
// pick a point on the window openings, uniformly by area
static float3 portalPoint(constant Uni& U, thread uint& seed) {
  float u = rnd(seed) * U.misc2.x; uint k = 0; uint np = U.counts.y;
  for (uint i = 0; i < np; i++) { float4 q = U.portal[i]; float a = (q.y - q.x) * (q.w - q.z); k = i; if (u < a) break; u -= a; }
  float4 q = U.portal[k];
  return float3(mix(q.x, q.y, rnd(seed)), mix(q.z, q.w, rnd(seed)), U.roomHi.z);
}
static bool throughPortal(float3 p, constant Uni& U) { // does the sun ray from p leave through a window opening?
  float3 s = U.sunDir.xyz; float t = (U.roomHi.z - p.z) / s.z; if (t < 0) return false;
  float3 q = p + s * t; uint np = U.counts.y;
  for (uint i = 0; i < np; i++) { float4 r = U.portal[i]; if (q.x > r.x && q.x < r.y && q.y > r.z && q.y < r.w) return true; }
  return false;
}
// irradiance from the sky through the windows, one sample; n is the receiving normal.
// mis = true: weighted against a cosine-sampled bounce ray (which then counts the sky it escapes to), so points right
// next to a window do not explode. mis = false: the geometric term is capped at pi instead (irradiance can never exceed pi * L).
static float3 portalE(thread Ctx& C, float3 p, float3 pOff, float3 n, constant Uni& U, thread uint& seed, uint mask, bool mis) {
  float3 q = portalPoint(U, seed); float3 dv = q - p; float r2 = dot(dv, dv); float3 w = dv / sqrt(r2);
  float cx = dot(n, w); if (cx <= 0.0f || w.z <= 1e-4f) return float3(0);
  if (occluded(C, pOff, w, 1e4f, mask)) return float3(0);
  float G = cx * w.z * U.misc2.x / r2;
  if (mis) { float pp = r2 / (U.misc2.x * w.z), pb = cx / PI; G *= pp / (pp + pb); } else G = min(G, PI);
  return sky(w, U) * G;
}
// weight for a cosine-sampled ray that left through a window (the other half of the MIS pair above)
static float skyRayWeight(float3 p, float3 wi, float nl, constant Uni& U) {
  if (wi.z <= 1e-4f) return 1.0f;
  float t = (U.roomHi.z - p.z) / wi.z; float pp = t * t / (U.misc2.x * wi.z), pb = nl / PI;
  return pb / (pp + pb);
}

// lighting of a surface reached by a bounce ray: emission + diffuse response to sun, sky and the bounce-light grid
static float3 shadeLite(thread Ctx& C, Surf y, float3 wIn, constant Uni& U, thread uint& seed, uint mask, bool misSun, float cosx, float rdist) {
  float3 s = U.sunDir.xyz; float3 L = y.emis; bool in = insideRoom(y.p, U);
  float3 kd = y.metal ? y.F0 * 0.6f : y.alb;
  // sun
  float3 sd = coneDir(s, U.sunDir.w, float2(rnd(seed), rnd(seed)));
  float ns = dot(y.n, sd);
  bool tr = any(y.trans > 0.0f);
  if (ns > 0.0f || tr) {
    float3 f = ns > 0.0f ? kd : y.trans;
    float3 po = y.p + y.ng * (ns > 0.0f ? EPS : -EPS);
    if (!occluded(C, po, sd, 1e4f, mask)) {
      float w = 1.0f;
      if (misSun && in && throughPortal(y.p, U)) {
        float cy = max(dot(y.n, -wIn), 1e-4f);
        float pv = (abs(ns) / U.misc.w) * rdist * rdist / cy; float pb = cosx / PI;
        w = pb / (pb + pv);
      }
      L += f * (1.0f / PI) * U.sunCol.xyz * abs(ns) * w;
    }
  }
  float3 E = float3(0);
  if (in) {
    E += portalE(C, y.p, y.p + y.ng * EPS, y.n, U, seed, mask, false);
    E += gridE(C, y.p + y.n * 0.05f, y.n, U);
    if (tr) L += y.trans * (1.0f / PI) * (portalE(C, y.p, y.p - y.ng * EPS, -y.n, U, seed, mask, false) + gridE(C, y.p - y.n * 0.05f, -y.n, U));
  } else {
    E += skyAmbientE(y.n, U);
  }
  L += kd * (1.0f / PI) * E;
  return L;
}

// explicit sampling of surfaces lit by the sun through the windows (diffuse lobe of x), MIS-weighted against the bounce ray
static float3 sunBounce(thread Ctx& C, Surf x, float3 fd, constant Uni& U, thread uint& seed, uint mask) {
  float3 s = U.sunDir.xyz;
  float3 q = portalPoint(U, seed);
  Hit h = traceWorld(C, q, -s, 0.0f, 60.0f, mask);
  if (!h.ok) return float3(0);
  Surf y = getSurf(C, h, q, -s, true, U, float3(0), float3(0), float3(0));
  float3 dv = y.p - x.p; float r2 = dot(dv, dv); if (r2 < 1e-8f) return float3(0);
  float r = sqrt(r2); float3 w = dv / r;
  float cx = dot(x.n, w); if (cx <= 0.0f || dot(x.ng, w) <= 0.0f) return float3(0);
  float cy = dot(y.n, -w);
  float3 fy = cy > 0.0f ? (y.metal ? y.F0 * 0.6f : y.alb) : y.trans;
  float acy = abs(cy); if (acy < 1e-4f || !any(fy > 0.0f)) return float3(0);
  if (occluded(C, x.p + x.ng * EPS, w, r - 2.0f * EPS, mask)) return float3(0);
  float nys = max(dot(y.n, s), 1e-4f);
  float pv = (nys / U.misc.w) * r2 / acy, pb = cx / PI;
  float wgt = pv / (pv + pb);
  return fd * cx * U.sunCol.xyz * fy * (1.0f / PI) * (U.misc.w * acy / r2) * wgt;
}

// ---------------------------------------------------------------- render kernel
kernel void render(uint2 tid [[thread_position_in_grid]],
                   device float4* acc [[buffer(0)]],
                   constant Uni& U [[buffer(1)]],
                   instance_acceleration_structure tlas [[buffer(2)]],
                   const device uint* idx [[buffer(3)]],
                   const device packed_float3* pos [[buffer(4)]],
                   const device packed_float3* nrm [[buffer(5)]],
                   const device packed_float3* tex [[buffer(6)]],
                   const device TriA* tri [[buffer(7)]],
                   const device Inst* inst [[buffer(8)]],
                   primitive_acceleration_structure shellAS [[buffer(9)]],
                   device float4* aov0 [[buffer(10)]],
                   device float4* aov1 [[buffer(11)]],
                   device float4* accE [[buffer(12)]],
                   texture3d<float> g0 [[texture(0)]], texture3d<float> g1 [[texture(1)]], texture3d<float> g2 [[texture(2)]],
                   texture3d<float> g3 [[texture(3)]], texture3d<float> g4 [[texture(4)]], texture3d<float> g5 [[texture(5)]])
{
  // Output is split in two so the soft part can be filtered: acc = everything sharp (sun, speculars, emission, fog),
  // accE = soft diffuse light (sky through the windows + bounce light) *before* it is multiplied by the surface colour.
  // The host filters accE guided by normal/depth and then multiplies by the averaged surface colour in aov0.
  uint W = U.dims.x, H = U.dims.y;
  if (tid.x >= W || tid.y >= H) return;
  Ctx C; C.tlas = tlas; C.shell = shellAS; C.idx = idx; C.pos = pos; C.nrm = nrm; C.tex = tex; C.tri = tri; C.inst = inst;
  C.g0 = g0; C.g1 = g1; C.g2 = g2; C.g3 = g3; C.g4 = g4; C.g5 = g5;
  uint pix = tid.x + tid.y * W;
  uint seed = pcg(pix * 9781u + U.counts.x * 6271u + 17u);
  float3 s = U.sunDir.xyz;
  float3 sum = float3(0), sAlb = float3(0), sNrm = float3(0), sumE = float3(0); float sDepth = 0, sLum2 = 0, nE = 0;
  uint spp = U.counts.z; uint mode = U.dims.z;
  for (uint si = 0; si < spp; si++) {
    // ---- camera sample: time, pixel (tent filter), lens
    float tt = rnd(seed);
    float3 eye = mix(U.camP0.xyz, U.camP1.xyz, tt);
    float3 cR = normalize(mix(U.camR0.xyz, U.camR1.xyz, tt)), cU = normalize(mix(U.camU0.xyz, U.camU1.xyz, tt)), cF = normalize(mix(U.camF0.xyz, U.camF1.xyz, tt));
    float j1 = rnd(seed), j2 = rnd(seed);
    float2 jit = float2(j1 < 0.5f ? sqrt(2.0f * j1) - 1.0f : 1.0f - sqrt(2.0f - 2.0f * j1), j2 < 0.5f ? sqrt(2.0f * j2) - 1.0f : 1.0f - sqrt(2.0f - 2.0f * j2)) * 0.85f;
    float2 pn = (float2(tid) + 0.5f + jit) / float2(W, H) * 2.0f - 1.0f; pn.y = -pn.y * U.lens.w;
    float rr = length(pn); float th = 2.0f * atan(rr * U.lens.x);
    float3 dc = rr > 1e-6f ? float3(sin(th) * pn.x / rr, sin(th) * pn.y / rr, cos(th)) : float3(0, 0, 1);
    float la = sqrt(rnd(seed)) * U.lens.y, lp = 2.0f * PI * rnd(seed); float2 lo = float2(cos(lp), sin(lp) * 0.85f) * la;
    float3 pf = dc * (U.lens.z / max(dc.z, 0.05f));
    float3 oc = float3(lo.x, lo.y, 0); float3 dcl = normalize(pf - oc);
    float3 o = eye + cR * oc.x + cU * oc.y; float3 d = normalize(cR * dcl.x + cU * dcl.y + cF * dcl.z);

    // ---- primary hit: the robot's own shell (camera space) first, then the world
    Hit h; h.ok = false; float tmax = 1e4f;
    if (U.misc2.z > 0.5f) {
      intersector<triangle_data> si2; si2.assume_geometry_type(geometry_type::triangle); si2.force_opacity(forced_opacity::opaque);
      ray r2; r2.origin = oc; r2.direction = dcl; r2.min_distance = 1e-4f; r2.max_distance = 2.0f;
      intersection_result<triangle_data> hs = si2.intersect(r2, C.shell);
      if (hs.type != intersection_type::none) { h.ok = true; h.shell = true; h.t = hs.distance; h.prim = hs.primitive_id; h.inst = 0; h.bc = hs.triangle_barycentric_coord; tmax = hs.distance; }
    }
    Hit hw = traceWorld(C, o, d, 1e-4f, tmax, MASK_WORLD);
    if (hw.ok) h = hw;

    float3 L = float3(0); float3 fAlb = float3(0), fNrm = float3(0); float fDepth = 60.0f;
    float3 Lind = float3(0), E = float3(0); bool dv = false;
    if (!h.ok) {
      L = sky(d, U);
      float cs = dot(d, s); if (cs > 0.99990f) L += U.sunCol.xyz * 60.0f;
    } else {
      Surf x = getSurf(C, h, o, d, false, U, cR, cU, cF);
      uint mask = h.shell ? MASK_WORLD : (MASK_WORLD | MASK_ROBOT);
      float3 v = -d; bool in = insideRoom(x.p, U);
      float nv = max(dot(x.n, v), 1e-4f);
      float3 Fv = fresnel(x.F0, nv);
      float3 fd = x.metal ? float3(0) : x.alb * (1.0f - Fv) * (1.0f / PI);
      float3 kd = x.metal ? float3(0) : x.alb * (1.0f - Fv);
      fAlb = kd; fNrm = x.n; fDepth = h.t; dv = !x.metal;
      L = x.emis;
      bool tr = any(x.trans > 0.0f);
      // sun, both lobes
      float3 sd = coneDir(s, U.sunDir.w, float2(rnd(seed), rnd(seed)));
      float ns = dot(x.n, sd);
      if (ns > 0.0f) {
        if (dot(x.ng, sd) > 0.0f && !occluded(C, x.p + x.ng * EPS, sd, 1e4f, mask)) L += U.sunCol.xyz * (fd + specEval(x, v, sd)) * ns;
      } else if (tr) {
        if (!occluded(C, x.p - x.ng * EPS, sd, 1e4f, mask)) L += U.sunCol.xyz * x.trans * (1.0f / PI) * (-ns);
      }
      if (in) {
        // sky through the windows (diffuse lobe; the specular lobe sees the sky through its bounce ray)
        if (!x.metal) E += portalE(C, x.p, x.p + x.ng * EPS, x.n, U, seed, mask, true) * (1.0f / PI);
        if (tr) L += x.trans * (1.0f / PI) * portalE(C, x.p, x.p - x.ng * EPS, -x.n, U, seed, mask, false);
        // sunlit surfaces as a light source
        if (!x.metal) E += sunBounce(C, x, float3(1.0f / PI), U, seed, mask);
      }
      // one bounce. Matte surfaces skip the specular bounce (their highlight comes from the sun term above):
      // it would only add noise for a few percent of the light.
      float pS = x.metal ? 1.0f : clamp(lum(Fv) / (lum(Fv) + lum(x.alb) * (1.0f - lum(Fv)) + 1e-4f), 0.06f, 0.94f);
      if (!x.metal && x.rough > 0.55f) pS = 0.0f;
      float3 t1, b1; basis(x.n, t1, b1);
      if (rnd(seed) < pS) {
        float a = max(x.rough * x.rough, 0.003f);
        float3 Ve = float3(dot(v, t1), dot(v, b1), dot(v, x.n));
        float3 hh = sampleVNDF(Ve, a, float2(rnd(seed), rnd(seed)));
        float3 hw2 = normalize(t1 * hh.x + b1 * hh.y + x.n * hh.z);
        float3 wi = reflect(-v, hw2);
        float nl = dot(x.n, wi);
        if (nl > 0.0f && dot(x.ng, wi) > 0.0f) {
          float3 wgt = fresnel(x.F0, dot(v, hw2)) * G1(nl, a) / pS;
          Hit hb = traceWorld(C, x.p + x.ng * EPS, wi, 0.0f, 1e4f, mask);
          if (!hb.ok) Lind += wgt * sky(wi, U);
          else { Surf y = getSurf(C, hb, x.p + x.ng * EPS, wi, true, U, float3(0), float3(0), float3(0)); Lind += wgt * shadeLite(C, y, wi, U, seed, mask, false, 1.0f, hb.t); }
        }
      } else {
        float u1 = rnd(seed), u2 = rnd(seed); float r = sqrt(u1), ph = 2.0f * PI * u2;
        float3 wi = normalize(t1 * (r * cos(ph)) + b1 * (r * sin(ph)) + x.n * sqrt(max(0.0f, 1.0f - u1)));
        float nl = dot(x.n, wi);
        if (nl > 0.0f && dot(x.ng, wi) > 0.0f) {
          float wgt = 1.0f / (1.0f - pS);
          Hit hb = traceWorld(C, x.p + x.ng * EPS, wi, 0.0f, 1e4f, mask);
          if (!hb.ok) { E += wgt * sky(wi, U) * (in ? skyRayWeight(x.p, wi, nl, U) : 1.0f); }
          else { Surf y = getSurf(C, hb, x.p + x.ng * EPS, wi, true, U, float3(0), float3(0), float3(0)); E += wgt * shadeLite(C, y, wi, U, seed, mask, in, nl, hb.t); }
        }
      }
      if (mode == 1) { L = x.alb + x.F0 * 0.5f; Lind = float3(0); E = float3(0); }
      if (mode == 2) { L = x.n * 0.5f + 0.5f; Lind = float3(0); E = float3(0); }
      if (mode == 3) { L = in ? gridE(C, x.p + x.n * 0.05f, x.n, U) * x.alb / PI : float3(0); Lind = float3(0); E = float3(0); }
      if (mode == 4) { Lind = float3(0); E = float3(0); }
      if (mode == 5) { L = float3(0); }
    }
    // sunlight scattered by dusty air along the camera ray
    if (U.misc.z > 0.0f && mode == 0) {
      float tf = min(h.ok ? h.t : 14.0f, 14.0f); float3 pfog = o + d * (tf * rnd(seed));
      if (insideRoom(pfog, U)) {
        float3 sd2 = coneDir(s, U.sunDir.w, float2(rnd(seed), rnd(seed)));
        if (!occluded(C, pfog, sd2, 1e4f, MASK_WORLD | MASK_ROBOT)) {
          float g = 0.72f; float cs = dot(d, s); float ph = (1.0f - g * g) / (4.0f * PI * pow(1.0f + g * g - 2.0f * g * cs, 1.5f));
          L += U.sunCol.xyz * (U.misc.z * tf * (ph + 0.012f));
        }
      }
    }
    float li = lum(Lind); if (li > U.misc.y) Lind *= U.misc.y / li;
    L += Lind;
    float ld = lum(L); if (ld > U.misc.y * 12.0f) L *= U.misc.y * 12.0f / ld;
    if (any(isnan(L)) || any(isinf(L))) L = float3(0);
    float le = lum(E); if (le > U.misc.y * 2.0f) E *= U.misc.y * 2.0f / le;
    if (any(isnan(E)) || any(isinf(E))) E = float3(0);
    sum += L; sAlb += fAlb; sNrm += fNrm; sDepth += fDepth;
    if (dv) { sumE += E; nE += 1.0f; sLum2 += lum(E) * lum(E); }
  }
  acc[pix] += float4(sum, float(spp));
  aov0[pix] += float4(sAlb, sDepth);
  aov1[pix] += float4(sNrm, sLum2);
  accE[pix] += float4(sumE, nE);
}

// ---------------------------------------------------------------- filter for the soft diffuse light
// Edge-avoiding a-trous wavelet filter (the spatial half of SVGF): neighbours count when they share the normal, lie on
// the same plane and differ by less than the pixel's own noise level, so shadows that are really there survive.
struct DP { uint W, H, step, pad; };
kernel void dnPrep(uint2 tid [[thread_position_in_grid]],
                   const device float4* accE [[buffer(0)]], const device float4* aov0 [[buffer(1)]], const device float4* aov1 [[buffer(2)]],
                   const device float4* acc [[buffer(3)]], device float4* oE [[buffer(4)]], device float4* oG [[buffer(5)]], constant DP& P [[buffer(6)]])
{
  if (tid.x >= P.W || tid.y >= P.H) return; uint i = tid.x + tid.y * P.W;
  float4 e = accE[i]; float spp = max(acc[i].w, 1.0f); float n = e.w;
  if (n < 0.5f) { oE[i] = float4(0); oG[i] = float4(0, 0, 0, -1); return; }
  float3 E = e.xyz / n; float l = lum(E);
  float var = max(aov1[i].w / n - l * l, 0.0f) / n;
  float3 nn = aov1[i].xyz; float ln = length(nn); nn = ln > 1e-6f ? nn / ln : float3(0, 1, 0);
  oE[i] = float4(E, var); oG[i] = float4(nn, aov0[i].w / spp);
}
kernel void dnAtrous(uint2 tid [[thread_position_in_grid]],
                     const device float4* iE [[buffer(0)]], const device float4* G [[buffer(1)]], device float4* oE [[buffer(2)]], constant DP& P [[buffer(3)]])
{
  int W = int(P.W), H = int(P.H); int x = int(tid.x), y = int(tid.y); if (x >= W || y >= H) return;
  int i = x + y * W; float4 gp = G[i]; float4 ep = iE[i];
  if (gp.w < 0.0f) { oE[i] = ep; return; }
  float vs = 0, vw = 0;
  for (int dy = -1; dy <= 1; dy++) for (int dx = -1; dx <= 1; dx++) {
    int q = clamp(x + dx, 0, W - 1) + clamp(y + dy, 0, H - 1) * W; if (G[q].w < 0.0f) continue;
    float w = (dx == 0 ? 2.0f : 1.0f) * (dy == 0 ? 2.0f : 1.0f); vs += w * iE[q].w; vw += w; }
  float sd = sqrt(max(vs / max(vw, 1.0f), 0.0f)); float lp = lum(ep.xyz);
  float za = G[min(x + 1, W - 1) + y * W].w, zb = G[max(x - 1, 0) + y * W].w, zc = G[x + min(y + 1, H - 1) * W].w, zd = G[x + max(y - 1, 0) * W].w;
  float gzx = min(za < 0.0f ? 1e9f : abs(za - gp.w), zb < 0.0f ? 1e9f : abs(zb - gp.w)); if (gzx > 1e8f) gzx = 0.0f;
  float gzy = min(zc < 0.0f ? 1e9f : abs(zc - gp.w), zd < 0.0f ? 1e9f : abs(zd - gp.w)); if (gzy > 1e8f) gzy = 0.0f;
  const float kw[3] = {0.375f, 0.25f, 0.0625f};
  float w0 = kw[0] * kw[0]; float3 sum = ep.xyz * w0; float wsum = w0, vsum = ep.w * w0 * w0;
  int st = int(P.step); float sl = 1.0f / (4.0f * sd + 0.02f * lp + 1e-5f);
  for (int dy = -2; dy <= 2; dy++) for (int dx = -2; dx <= 2; dx++) {
    if (dx == 0 && dy == 0) continue;
    int qx = x + dx * st, qy = y + dy * st; if (qx < 0 || qy < 0 || qx >= W || qy >= H) continue;
    int q = qx + qy * W; float4 gq = G[q]; if (gq.w < 0.0f) continue; float4 eq = iE[q];
    float cn = max(dot(gp.xyz, gq.xyz), 0.0f); float c2 = cn * cn, c4 = c2 * c2, c8 = c4 * c4, c16 = c8 * c8; float wn = c16 * c16 * c16;
    float wz = exp(-abs(gp.w - gq.w) / ((gzx * float(abs(dx)) + gzy * float(abs(dy))) * float(st) * 1.5f + 0.004f * gp.w + 1e-4f));
    float wl = exp(-abs(lp - lum(eq.xyz)) * sl);
    float w = kw[abs(dx)] * kw[abs(dy)] * wn * wz * wl;
    sum += eq.xyz * w; wsum += w; vsum += eq.w * w * w;
  }
  oE[i] = float4(sum / wsum, vsum / (wsum * wsum));
}

// ---------------------------------------------------------------- bounce-light bake
// each thread is one voxel; it gathers incoming bounced radiance over the sphere into six directional irradiances
kernel void bake(uint2 tid [[thread_position_in_grid]],
                 device float4* gacc [[buffer(0)]],
                 constant Uni& U [[buffer(1)]],
                 instance_acceleration_structure tlas [[buffer(2)]],
                 const device uint* idx [[buffer(3)]],
                 const device packed_float3* pos [[buffer(4)]],
                 const device packed_float3* nrm [[buffer(5)]],
                 const device packed_float3* tex [[buffer(6)]],
                 const device TriA* tri [[buffer(7)]],
                 const device Inst* inst [[buffer(8)]],
                 primitive_acceleration_structure shellAS [[buffer(9)]],
                 texture3d<float> g0 [[texture(0)]], texture3d<float> g1 [[texture(1)]], texture3d<float> g2 [[texture(2)]],
                 texture3d<float> g3 [[texture(3)]], texture3d<float> g4 [[texture(4)]], texture3d<float> g5 [[texture(5)]])
{
  uint nx = U.grid.x, ny = U.grid.y, nz = U.grid.z;
  if (tid.x >= nx * nz || tid.y >= ny) return;
  uint ix = tid.x % nx, iz = tid.x / nx, iy = tid.y;
  uint vox = ix + nx * (iy + ny * iz);
  Ctx C; C.tlas = tlas; C.shell = shellAS; C.idx = idx; C.pos = pos; C.nrm = nrm; C.tex = tex; C.tri = tri; C.inst = inst;
  C.g0 = g0; C.g1 = g1; C.g2 = g2; C.g3 = g3; C.g4 = g4; C.g5 = g5;
  uint seed = pcg(vox * 7919u + U.counts.x * 104729u + 3u);
  float3 cell = 1.0f / (U.gridInv.xyz * float3(nx, ny, nz));
  float3 e[6]; for (int k = 0; k < 6; k++) e[k] = float3(0);
  float inside = 0; uint K = U.grid.w;
  for (uint k = 0; k < K; k++) {
    float3 o = U.gridLo.xyz + (float3(ix, iy, iz) + 0.5f + (float3(rnd(seed), rnd(seed), rnd(seed)) - 0.5f) * 0.6f) * cell;
    float z = 1.0f - 2.0f * rnd(seed), ph = 2.0f * PI * rnd(seed), r = sqrt(max(0.0f, 1.0f - z * z));
    float3 d = float3(r * cos(ph), z, r * sin(ph));
    Hit h = traceWorld(C, o, d, 0.0f, 1e4f, MASK_WORLD);
    if (!h.ok) continue;
    Surf y = getSurf(C, h, o, d, true, U, float3(0), float3(0), float3(0));
    bool twoSided = y.mat == M_RUG || y.mat == M_LEAF || y.mat == M_CURTAIN || y.mat == M_FOLIAGE;
    if (y.back && !twoSided) { inside += 1.0f; continue; }
    float3 L = shadeLite(C, y, d, U, seed, MASK_WORLD, false, 1.0f, h.t);
    float l = lum(L); if (l > 40.0f) L *= 40.0f / l;
    e[0] += L * max(d.x, 0.0f); e[1] += L * max(-d.x, 0.0f); e[2] += L * max(d.y, 0.0f);
    e[3] += L * max(-d.y, 0.0f); e[4] += L * max(d.z, 0.0f); e[5] += L * max(-d.z, 0.0f);
  }
  for (int k = 0; k < 6; k++) gacc[vox * 7 + k] += float4(e[k], 0);
  gacc[vox * 7 + 6] += float4(inside, float(K), 0, 0);
}
