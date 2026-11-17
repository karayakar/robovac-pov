// base.h — small math, RNG and mesh container shared by the whole host
#pragma once
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <string>
#include <algorithm>
#include <functional>
typedef uint32_t u32;
static const float PI = 3.14159265358979f;

struct V3 { float x = 0, y = 0, z = 0; V3() {} V3(float a, float b, float c) : x(a), y(b), z(c) {} };
static inline V3 operator+(V3 a, V3 b) { return V3(a.x + b.x, a.y + b.y, a.z + b.z); }
static inline V3 operator-(V3 a, V3 b) { return V3(a.x - b.x, a.y - b.y, a.z - b.z); }
static inline V3 operator-(V3 a) { return V3(-a.x, -a.y, -a.z); }
static inline V3 operator*(V3 a, float s) { return V3(a.x * s, a.y * s, a.z * s); }
static inline V3 operator*(float s, V3 a) { return V3(a.x * s, a.y * s, a.z * s); }
static inline V3 operator*(V3 a, V3 b) { return V3(a.x * b.x, a.y * b.y, a.z * b.z); }
static inline V3 operator/(V3 a, float s) { return V3(a.x / s, a.y / s, a.z / s); }
static inline V3& operator+=(V3& a, V3 b) { a = a + b; return a; }
static inline float dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static inline V3 cross(V3 a, V3 b) { return V3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x); }
static inline float len(V3 a) { return sqrtf(dot(a, a)); }
static inline V3 norm(V3 a) { float l = len(a); return l > 1e-20f ? a / l : V3(0, 1, 0); }
static inline float clampf(float x, float a, float b) { return x < a ? a : (x > b ? b : x); }
static inline float lerpf(float a, float b, float t) { return a + (b - a) * t; }
static inline V3 lerp(V3 a, V3 b, float t) { return a + (b - a) * t; }
static inline float smooth(float a, float b, float x) { float t = clampf((x - a) / (b - a), 0, 1); return t * t * (3 - 2 * t); }
static inline float rad(float deg) { return deg * PI / 180.0f; }

// 3x4 affine transform: p' = R p + t   (R stored row-major in m[0..8], t in m[9..11])
struct M34 {
  float r[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1}; V3 t;
  V3 pt(V3 p) const { return V3(r[0] * p.x + r[1] * p.y + r[2] * p.z + t.x, r[3] * p.x + r[4] * p.y + r[5] * p.z + t.y, r[6] * p.x + r[7] * p.y + r[8] * p.z + t.z); }
  V3 dir(V3 p) const { return V3(r[0] * p.x + r[1] * p.y + r[2] * p.z, r[3] * p.x + r[4] * p.y + r[5] * p.z, r[6] * p.x + r[7] * p.y + r[8] * p.z); }
};
static inline M34 operator*(const M34& a, const M34& b) { // apply b first, then a
  M34 o;
  for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) o.r[i * 3 + j] = a.r[i * 3] * b.r[j] + a.r[i * 3 + 1] * b.r[3 + j] + a.r[i * 3 + 2] * b.r[6 + j];
  o.t = a.pt(b.t); return o;
}
static inline M34 T(V3 t) { M34 m; m.t = t; return m; }
static inline M34 T(float x, float y, float z) { return T(V3(x, y, z)); }
static inline M34 S(float sx, float sy, float sz) { M34 m; m.r[0] = sx; m.r[4] = sy; m.r[8] = sz; return m; }
static inline M34 S(float s) { return S(s, s, s); }
static inline M34 RX(float a) { M34 m; float c = cosf(a), s = sinf(a); m.r[4] = c; m.r[5] = -s; m.r[7] = s; m.r[8] = c; return m; }
static inline M34 RY(float a) { M34 m; float c = cosf(a), s = sinf(a); m.r[0] = c; m.r[2] = s; m.r[6] = -s; m.r[8] = c; return m; }
static inline M34 RZ(float a) { M34 m; float c = cosf(a), s = sinf(a); m.r[0] = c; m.r[1] = -s; m.r[3] = s; m.r[4] = c; return m; }
// frame whose local +Y axis points along d (for laying cylinders/lathes along a direction)
static inline M34 alignY(V3 d) {
  V3 y = norm(d), h = fabsf(y.y) < 0.99f ? V3(0, 1, 0) : V3(1, 0, 0);
  V3 x = norm(cross(h, y)), z = cross(x, y);
  M34 m; m.r[0] = x.x; m.r[3] = x.y; m.r[6] = x.z; m.r[1] = y.x; m.r[4] = y.y; m.r[7] = y.z; m.r[2] = z.x; m.r[5] = z.y; m.r[8] = z.z; return m;
}
static inline M34 inverseRigid(const M34& a) { // valid for rotation + translation only
  M34 o; for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) o.r[i * 3 + j] = a.r[j * 3 + i];
  o.t = -o.dir(a.t); return o;
}

static inline u32 pcg(u32 v) { u32 s = v * 747796405u + 2891336453u; u32 w = ((s >> ((s >> 28u) + 4u)) ^ s) * 277803737u; return (w >> 22u) ^ w; }
struct Rng {
  u32 s; Rng(u32 seed = 1) : s(pcg(seed)) {}
  float f() { s = pcg(s); return (s >> 8) * (1.0f / 16777216.0f); }
  float r(float a, float b) { return a + (b - a) * f(); }
  int i(int n) { return std::min(n - 1, (int)(f() * n)); }
  float g() { float a = 0; for (int k = 0; k < 4; k++) a += f(); return (a - 2.0f) * 1.7320508f; } // ~N(0,1)
};
// value noise on CPU (for shapes)
static inline float hash3i(int x, int y, int z) { return (pcg((u32)x * 73856093u ^ (u32)y * 19349663u ^ (u32)z * 83492791u) >> 8) * (1.0f / 16777216.0f); }
static inline float vnoise(V3 p) {
  int ix = (int)floorf(p.x), iy = (int)floorf(p.y), iz = (int)floorf(p.z);
  float fx = p.x - ix, fy = p.y - iy, fz = p.z - iz;
  fx = fx * fx * (3 - 2 * fx); fy = fy * fy * (3 - 2 * fy); fz = fz * fz * (3 - 2 * fz);
  float c[8]; for (int k = 0; k < 8; k++) c[k] = hash3i(ix + (k & 1), iy + ((k >> 1) & 1), iz + ((k >> 2) & 1));
  float x0 = lerpf(c[0], c[1], fx), x1 = lerpf(c[2], c[3], fx), x2 = lerpf(c[4], c[5], fx), x3 = lerpf(c[6], c[7], fx);
  return lerpf(lerpf(x0, x1, fy), lerpf(x2, x3, fy), fz);
}
static inline float fbm(V3 p, int oct) { float a = 0.5f, s = 0; for (int i = 0; i < oct; i++) { s += a * vnoise(p); p = p * 2.03f + V3(17.1f, 3.3f, 9.7f); a *= 0.5f; } return s; }

// material ids (must match render.metal)
enum Mat : u32 {
  M_FLOOR = 0, M_WOOD, M_PAINT, M_TRIM, M_FABRIC, M_RUG, M_PLASTIC, M_METAL, M_CERAMIC, M_LEAF, M_ROBOT, M_EMIT, M_MATTE,
  M_FACADE, M_FOLIAGE, M_RUBBER, M_CONCRETE, M_CURTAIN, M_BOOK, M_ART, M_SKIN, M_SOCKET
};
// colour is authored in sRGB 0..1; alpha is a free per-material parameter (roughness, random, ...)
static inline u32 RGBA(float r, float g, float b, float a = 0.5f) {
  auto q = [](float v) { return (u32)clampf(v * 255.0f + 0.5f, 0, 255); };
  return q(r) | (q(g) << 8) | (q(b) << 16) | (q(a) << 24);
}

struct Mesh {
  std::vector<V3> p, n, t;      // per vertex: position, normal, texture/object coordinate
  std::vector<u32> idx;         // 3 per triangle
  std::vector<u32> tmat, tcol;  // per triangle
  u32 mat = M_MATTE, col = 0xffffffff;
  u32 v(V3 pos, V3 nrm, V3 tex) { p.push_back(pos); n.push_back(nrm); t.push_back(tex); return (u32)p.size() - 1; }
  void tri(u32 a, u32 b, u32 c) { idx.push_back(a); idx.push_back(b); idx.push_back(c); tmat.push_back(mat); tcol.push_back(col); }
  void quad(u32 a, u32 b, u32 c, u32 d) { tri(a, b, c); tri(a, c, d); }
  size_t ntri() const { return tmat.size(); }
  void set(u32 m, u32 c) { mat = m; col = c; }
};
// append src to dst through a transform (normals are rotated with the linear part; fine for rigid + uniform scale,
// and re-normalised for mild non-uniform scale). Texture coordinates stay in the source's object space.
static inline void add(Mesh& dst, const Mesh& src, const M34& xf = M34()) {
  u32 base = (u32)dst.p.size();
  // normals go through the cofactor matrix (inverse transpose up to scale), so non-uniform scale is handled
  const float* r = xf.r;
  float c[9] = {r[4] * r[8] - r[5] * r[7], r[5] * r[6] - r[3] * r[8], r[3] * r[7] - r[4] * r[6],
                r[2] * r[7] - r[1] * r[8], r[0] * r[8] - r[2] * r[6], r[1] * r[6] - r[0] * r[7],
                r[1] * r[5] - r[2] * r[4], r[2] * r[3] - r[0] * r[5], r[0] * r[4] - r[1] * r[3]};
  float det = r[0] * c[0] + r[1] * c[1] + r[2] * c[2]; float sg = det < 0 ? -1.0f : 1.0f;
  for (size_t i = 0; i < src.p.size(); i++) {
    V3 n = src.n[i];
    dst.p.push_back(xf.pt(src.p[i]));
    dst.n.push_back(norm(V3(c[0] * n.x + c[1] * n.y + c[2] * n.z, c[3] * n.x + c[4] * n.y + c[5] * n.z, c[6] * n.x + c[7] * n.y + c[8] * n.z) * sg));
    dst.t.push_back(src.t[i]);
  }
  if (det < 0) for (size_t i = 0; i + 2 < src.idx.size(); i += 3) { dst.idx.push_back(base + src.idx[i]); dst.idx.push_back(base + src.idx[i + 2]); dst.idx.push_back(base + src.idx[i + 1]); }
  else for (u32 i : src.idx) dst.idx.push_back(base + i);
  dst.tmat.insert(dst.tmat.end(), src.tmat.begin(), src.tmat.end());
  dst.tcol.insert(dst.tcol.end(), src.tcol.begin(), src.tcol.end());
}
// make a closed chunk (triangles from firstTri on) wind outward: positive signed volume
static inline void fixWinding(Mesh& m, size_t firstTri) {
  double vol = 0; V3 c0 = m.p[m.idx[firstTri * 3]];
  for (size_t i = firstTri; i < m.ntri(); i++) { V3 a = m.p[m.idx[i * 3]] - c0, b = m.p[m.idx[i * 3 + 1]] - c0, c = m.p[m.idx[i * 3 + 2]] - c0; vol += dot(a, cross(b, c)); }
  if (vol < 0) for (size_t i = firstTri; i < m.ntri(); i++) std::swap(m.idx[i * 3 + 1], m.idx[i * 3 + 2]);
}
