// scene.h — the apartment, modelled entirely in code. Units are metres; x east, y up, z south (windows on the south wall).
#pragma once
#include "geom.h"
#include "path.h"

struct Scene {
  Mesh world, dino, robot, shell; // static world; dinosaur (local, +x forward); robot body (robot-local); deck in camera space
  V3 sun = norm(V3(-0.36f, 0.60f, 0.72f));
  float portal[2][4] = {{1.00f, 2.10f, 0.10f, 2.30f}, {3.40f, 4.50f, 0.10f, 2.30f}};
  int nPortal = 2;
  DinoAnim dinoAnim;
  std::vector<V3> cereal;
};

static const u32 C_WALL = RGBA(0.91f, 0.89f, 0.85f), C_CEIL = RGBA(0.95f, 0.95f, 0.94f), C_TRIM = RGBA(0.94f, 0.94f, 0.92f);
static const u32 C_WALNUT = RGBA(0.40f, 0.26f, 0.17f, 0.0f), C_WALNUT_V = RGBA(0.40f, 0.26f, 0.17f, 1.0f);
static const u32 C_OAK = RGBA(0.74f, 0.58f, 0.40f, 0.0f), C_OAK_V = RGBA(0.74f, 0.58f, 0.40f, 1.0f);

static inline void boxMM(Mesh& m, V3 lo, V3 hi, float r = 0.003f, int seg = 1) { rboxMM(m, lo, hi, r, seg); }

// ---------------------------------------------------------------- room shell
static void buildRoom(Mesh& m, Scene& sc) {
  const float X = ROOM_X, Z = ROOM_Z, H = ROOM_H, PZ = ROOM_Z + WALL_T;
  m.set(M_PAINT, C_WALL);
  quad(m, V3(0, 0, 0), V3(X, 0, 0), V3(X, H, 0), V3(0, H, 0));           // north wall
  quad(m, V3(0, 0, 0), V3(0, H, 0), V3(0, H, Z), V3(0, 0, Z));           // west wall
  quad(m, V3(X, 0, 0), V3(X, 0, Z), V3(X, H, Z), V3(X, H, 0));           // east wall
  m.set(M_PAINT, C_CEIL);
  quad(m, V3(0, H, 0), V3(X, H, 0), V3(X, H, Z), V3(0, H, Z));           // ceiling
  m.set(M_MATTE, RGBA(0.05f, 0.04f, 0.03f));
  quad(m, V3(0, -0.016f, 0), V3(0, -0.016f, Z), V3(X, -0.016f, Z), V3(X, -0.016f, 0)); // subfloor under the planks
  // south wall with two tall window openings, built as solid pieces so the reveals exist
  m.set(M_PAINT, C_WALL);
  float xs[5] = {-0.3f, sc.portal[0][0], sc.portal[0][1], sc.portal[1][0], sc.portal[1][1]};
  float y0 = sc.portal[0][2], y1 = sc.portal[0][3];
  boxMM(m, V3(xs[0], -0.3f, Z), V3(xs[1], H + 0.3f, PZ), 0.002f);
  boxMM(m, V3(xs[2], -0.3f, Z), V3(xs[3], H + 0.3f, PZ), 0.002f);
  boxMM(m, V3(xs[4], -0.3f, Z), V3(X + 0.3f, H + 0.3f, PZ), 0.002f);
  for (int w = 0; w < 2; w++) {
    float a = sc.portal[w][0], b = sc.portal[w][1];
    m.set(M_PAINT, C_WALL);
    boxMM(m, V3(a, y1, Z), V3(b, H + 0.3f, PZ), 0.002f);                 // lintel
    m.set(M_TRIM, C_TRIM);
    boxMM(m, V3(a, -0.3f, Z - 0.012f), V3(b, y0, PZ + 0.02f), 0.004f);   // sill / threshold
    // window frame in the middle of the reveal
    float zf0 = Z + 0.09f, zf1 = Z + 0.15f, fw = 0.055f;
    boxMM(m, V3(a, y0, zf0), V3(a + fw, y1, zf1), 0.004f); boxMM(m, V3(b - fw, y0, zf0), V3(b, y1, zf1), 0.004f);
    boxMM(m, V3(a, y1 - fw, zf0), V3(b, y1, zf1), 0.004f); boxMM(m, V3(a, y0, zf0), V3(b, y0 + fw + 0.02f, zf1), 0.004f);
    float mx = (a + b) * 0.5f;
    boxMM(m, V3(mx - 0.035f, y0, zf0), V3(mx + 0.035f, y1, zf1), 0.004f); // meeting stiles of the double door
    boxMM(m, V3(a, 1.74f, zf0 + 0.01f), V3(b, 1.785f, zf1 - 0.01f), 0.003f); // glazing bar
    // handle
    m.set(M_METAL, RGBA(0.78f, 0.76f, 0.72f, 0.25f));
    cyl(m, V3(mx + 0.012f, 1.02f, zf0 - 0.002f), V3(mx + 0.012f, 1.02f, zf0 - 0.045f), 0.008f, 0.008f, 12);
    cyl(m, V3(mx + 0.012f, 1.02f, zf0 - 0.040f), V3(mx + 0.012f, 0.90f, zf0 - 0.040f), 0.007f, 0.006f, 12);
  }
  // baseboards
  m.set(M_TRIM, C_TRIM);
  const float bh = 0.075f, bt = 0.014f;
  boxMM(m, V3(0, 0, 0), V3(X, bh, bt), 0.004f, 2);                         // north
  boxMM(m, V3(0, 0, 0), V3(bt, bh, 0.40f), 0.004f, 2); boxMM(m, V3(0, 0, 1.40f), V3(bt, bh, Z), 0.004f, 2); // west, around the door
  boxMM(m, V3(X - bt, 0, 0), V3(X, bh, Z), 0.004f, 2);                     // east
  boxMM(m, V3(0, 0, Z - bt), V3(xs[1], bh, Z), 0.004f, 2); boxMM(m, V3(xs[2], 0, Z - bt), V3(xs[3], bh, Z), 0.004f, 2); boxMM(m, V3(xs[4], 0, Z - bt), V3(X, bh, Z), 0.004f, 2);
  // door on the west wall
  boxMM(m, V3(0, 0, 0.385f), V3(0.022f, 2.10f, 0.45f), 0.004f, 2); boxMM(m, V3(0, 0, 1.35f), V3(0.022f, 2.10f, 1.415f), 0.004f, 2);
  boxMM(m, V3(0, 2.05f, 0.385f), V3(0.022f, 2.115f, 1.415f), 0.004f, 2);
  boxMM(m, V3(-0.03f, 0.009f, 0.452f), V3(0.010f, 2.048f, 1.348f), 0.003f, 2);
  for (int k = 0; k < 2; k++) { float ya = k ? 1.12f : 0.20f, yb = k ? 1.90f : 1.00f; // raised panel mouldings
    boxMM(m, V3(0.008f, ya, 0.57f), V3(0.016f, yb, 1.23f), 0.006f, 2); }
  m.set(M_METAL, RGBA(0.72f, 0.70f, 0.66f, 0.3f));
  cyl(m, V3(0.010f, 1.03f, 1.27f), V3(0.055f, 1.03f, 1.27f), 0.009f, 0.009f, 12);
  cyl(m, V3(0.050f, 1.03f, 1.275f), V3(0.050f, 1.03f, 1.15f), 0.008f, 0.007f, 12);
  // wall sockets
  m.set(M_TRIM, RGBA(0.96f, 0.96f, 0.95f));
  boxMM(m, V3(3.68f, 0.21f, 0), V3(3.76f, 0.29f, 0.010f), 0.004f, 2);
  boxMM(m, V3(0, 0.21f, 3.50f), V3(0.010f, 0.29f, 3.58f), 0.004f, 2);
  boxMM(m, V3(X - 0.010f, 0.21f, 0.62f), V3(X, 0.29f, 0.70f), 0.004f, 2);
  // ceiling: pendant over the dining table
  m.set(M_RUBBER, RGBA(0.05f, 0.05f, 0.05f));
  cyl(m, V3(TABLE_X, 1.92f, TABLE_Z), V3(TABLE_X, H, TABLE_Z), 0.004f, 0.004f, 8);
  m.set(M_CERAMIC, RGBA(0.90f, 0.88f, 0.84f, 0.9f));
  ellipsoid(m, T(TABLE_X, 1.74f, TABLE_Z), V3(0.21f, 0.19f, 0.21f), 36, 18);
}

// ---------------------------------------------------------------- oak floor: individual planks with real seams
static void buildFloor(Mesh& m) {
  Rng g(11);
  int rows = (int)ceilf(ROOM_Z / PLANK_W);
  for (int r = 0; r < rows; r++) {
    float z0 = r * PLANK_W, z1 = std::min(z0 + PLANK_W, ROOM_Z);
    float x = -g.r(0.1f, 1.3f);
    while (x < ROOM_X) {
      float L = g.r(1.1f, 1.9f); float x0 = std::max(x, 0.0f), x1 = std::min(x + L, ROOM_X);
      if (x1 - x0 > 0.02f) {
        m.set(M_FLOOR, RGBA(1, 1, 1, g.f()));
        float gap = 0.00035f;
        rbox(m, T((x0 + x1) * 0.5f, -0.0075f + g.r(-0.00015f, 0.00015f), (z0 + z1) * 0.5f), V3((x1 - x0) * 0.5f - gap, 0.0075f, (z1 - z0) * 0.5f - gap), 0.0011f, 1);
      }
      x += L;
    }
  }
}

// ---------------------------------------------------------------- rug: backing + one ribbon per tuft + fringe
static void buildRug(Mesh& m) {
  Rng g(23);
  float W = RUG_X1 - RUG_X0, D = RUG_Z1 - RUG_Z0;
  m.set(M_RUG, RGBA(1, 1, 1, 0.5f));
  { // backing: thin slab; tex = rug-local position, height fraction 0 (reads as the dark base of the pile)
    Mesh b; b.set(M_RUG, RGBA(1, 1, 1, 0.5f));
    rbox(b, T(0, 0, 0), V3(W * 0.5f, 0.0025f, D * 0.5f), 0.002f, 1);
    for (auto& t : b.t) t = V3(t.x + W * 0.5f, 0.0f, t.z + D * 0.5f);
    add(m, b, T(RUG_X0 + W * 0.5f, 0.0025f, RUG_Z0 + D * 0.5f));
  }
  const float sp = 0.00195f; int nx = (int)(W / sp), nz = (int)(D / sp);
  m.p.reserve(m.p.size() + (size_t)nx * nz * 4); m.n.reserve(m.p.size() + (size_t)nx * nz * 4); m.t.reserve(m.p.size() + (size_t)nx * nz * 4);
  for (int j = 0; j < nz; j++) for (int i = 0; i < nx; i++) {
    float lx = (i + g.f()) * sp, lz = (j + g.f()) * sp;
    float edge = std::min(std::min(lx, W - lx), std::min(lz, D - lz));
    float h = (0.0085f + 0.0045f * g.f()) * (0.55f + 0.45f * smooth(0.0f, 0.012f, edge));
    // slow "combed" lean field plus per-tuft randomness
    float lean = 0.0045f; V3 ln((vnoise(V3(lx * 5, 1, lz * 5)) - 0.5f) * 2 * lean + g.r(-0.0022f, 0.0022f), 0, (vnoise(V3(lx * 5, 9, lz * 5)) - 0.5f) * 2 * lean + g.r(-0.0022f, 0.0022f));
    float a = g.f() * PI; float w0 = 0.0016f, w1 = 0.0009f;
    V3 root(RUG_X0 + lx, 0.004f, RUG_Z0 + lz), tip = root + V3(0, h, 0) + ln;
    m.col = RGBA(1, 1, 1, g.f());
    for (int c = 0; c < 2; c++) { // two crossed blades, so a tuft never disappears edge-on
      V3 side(cosf(a + c * PI * 0.5f), 0, sinf(a + c * PI * 0.5f));
      V3 fn = norm(cross(side, tip - root)); V3 nn = norm(fn * 0.45f + V3(0, 1, 0));
      u32 v0 = m.v(root - side * w0, nn, V3(lx, 0.15f, lz)), v1 = m.v(root + side * w0, nn, V3(lx, 0.15f, lz));
      u32 v2 = m.v(tip + side * w1, nn, V3(lx, 1.0f, lz)), v3 = m.v(tip - side * w1, nn, V3(lx, 1.0f, lz));
      m.quad(v0, v1, v2, v3);
    }
  }
  // fringe on the two short ends
  m.set(M_MATTE, RGBA(0.80f, 0.74f, 0.62f));
  for (int e = 0; e < 2; e++) for (float z = RUG_Z0 + 0.01f; z < RUG_Z1 - 0.01f; z += 0.0052f) {
    float dir = e ? 1.0f : -1.0f, x0 = e ? RUG_X1 - 0.002f : RUG_X0 + 0.002f; float L = g.r(0.038f, 0.052f), wob = g.r(-0.012f, 0.012f);
    std::vector<V3> c = {V3(x0, 0.0045f, z), V3(x0 + dir * L * 0.35f, 0.0030f, z + wob * 0.3f), V3(x0 + dir * L * 0.7f, 0.0016f, z + wob * 0.8f), V3(x0 + dir * L, 0.0013f, z + wob + g.r(-0.004f, 0.004f))};
    tube(m, spline(c, 3), std::vector<float>(10, 0.0011f), 5, false);
  }
}

// ---------------------------------------------------------------- furniture
static void buildSofa(Mesh& m) {
  const u32 fab = RGBA(0.57f, 0.56f, 0.52f, 0.3f);
  float x0 = SOFA_X0, x1 = SOFA_X1, z0 = SOFA_Z0, z1 = SOFA_Z1, c = SOFA_CLEAR;
  m.set(M_FABRIC, fab);
  rboxMM(m, V3(x0, c, z0), V3(x1, 0.315f, z1 - 0.02f), 0.018f, 3);                 // base
  rboxMM(m, V3(x0, 0.30f, z0), V3(x0 + 0.19f, 0.63f, z1 - 0.02f), 0.05f, 4);       // arms
  rboxMM(m, V3(x1 - 0.19f, 0.30f, z0), V3(x1, 0.63f, z1 - 0.02f), 0.05f, 4);
  rboxMM(m, V3(x0 + 0.17f, 0.30f, z0), V3(x1 - 0.17f, 0.82f, z0 + 0.22f), 0.05f, 4); // back
  float cw = (x1 - x0 - 0.38f) / 3;
  for (int i = 0; i < 3; i++) {
    float a = x0 + 0.19f + i * cw;
    rboxMM(m, V3(a + 0.004f, 0.305f, z0 + 0.20f), V3(a + cw - 0.004f, 0.47f, z1), 0.05f, 4);      // seat cushion
    Mesh b; b.set(M_FABRIC, fab); rbox(b, T(0, 0, 0), V3(cw * 0.5f - 0.006f, 0.19f, 0.085f), 0.07f, 4);
    add(m, b, T(a + cw * 0.5f, 0.66f, z0 + 0.30f) * RX(rad(-11)));                                  // back cushion
  }
  { Mesh b; b.set(M_FABRIC, RGBA(0.80f, 0.60f, 0.22f, 0.7f)); rbox(b, T(0, 0, 0), V3(0.21f, 0.21f, 0.07f), 0.068f, 4); add(m, b, T(x0 + 0.42f, 0.68f, z0 + 0.46f) * RY(rad(14)) * RX(rad(-22))); }
  { Mesh b; b.set(M_FABRIC, RGBA(0.34f, 0.44f, 0.52f, 0.1f)); rbox(b, T(0, 0, 0), V3(0.19f, 0.19f, 0.065f), 0.062f, 4); add(m, b, T(x1 - 0.45f, 0.66f, z0 + 0.48f) * RY(rad(-18)) * RX(rad(-20))); }
  m.set(M_MATTE, RGBA(0.035f, 0.035f, 0.035f));
  boxMM(m, V3(x0 + 0.02f, c - 0.004f, z0 + 0.02f), V3(x1 - 0.02f, c + 0.002f, z1 - 0.04f), 0.001f); // dust cover underneath
  m.set(M_WOOD, RGBA(0.33f, 0.21f, 0.14f, 1.0f));
  for (int i = 0; i < 2; i++) for (int j = 0; j < 2; j++) {
    float lx = i ? x1 - 0.075f : x0 + 0.075f, lz = j ? z1 - 0.10f : z0 + 0.075f;
    cyl(m, V3(lx + (i ? 0.012f : -0.012f), 0, lz + (j ? 0.012f : -0.012f)), V3(lx, c, lz), 0.016f, 0.026f, 20, 0.002f);
  }
}
static void buildCoffeeTable(Mesh& m) {
  float cx = 2.0f, cz = 1.85f, hw = 0.56f, hd = 0.28f, top = 0.42f;
  m.set(M_WOOD, C_WALNUT);
  rboxMM(m, V3(cx - hw, top - 0.034f, cz - hd), V3(cx + hw, top, cz + hd), 0.010f, 3);
  m.set(M_WOOD, C_WALNUT_V);
  for (int i = -1; i <= 1; i += 2) for (int j = -1; j <= 1; j += 2)
    cyl(m, V3(cx + i * (hw - 0.03f), RUG_H - 0.004f, cz + j * (hd - 0.02f)), V3(cx + i * (hw - 0.10f), top - 0.03f, cz + j * (hd - 0.07f)), 0.013f, 0.023f, 20, 0.002f);
  // a couple of things on top (seen from across the room)
  m.set(M_BOOK, RGBA(0.16f, 0.26f, 0.33f, 0.8f)); { Mesh b; b.set(m.mat, m.col); rbox(b, T(0, 0, 0), V3(0.14f, 0.012f, 0.10f), 0.002f, 1); add(m, b, T(cx - 0.2f, top + 0.012f, cz + 0.02f) * RY(rad(17))); }
  m.set(M_CERAMIC, RGBA(0.86f, 0.84f, 0.78f, 0.2f));
  lathe(m, T(cx + 0.25f, top, cz - 0.05f), Profile{{0, 0}, {0.034f, 0}, {0.040f, 0.004f}, {0.043f, 0.05f}, {0.042f, 0.088f}, {0.039f, 0.09f}, {0.037f, 0.088f}, {0.038f, 0.05f}, {0.034f, 0.008f}, {0, 0.008f}}, 28);
}
static void buildDining(Mesh& m) {
  float cx = TABLE_X, cz = TABLE_Z, hx = 0.45f, hz = 0.80f, top = 0.75f;
  m.set(M_WOOD, C_OAK);
  { Mesh b; b.set(M_WOOD, C_OAK); rbox(b, T(0, 0, 0), V3(hz, 0.017f, hx), 0.006f, 2); add(m, b, T(cx, top - 0.017f, cz) * RY(rad(90))); } // grain along z
  { Mesh b; b.set(M_WOOD, C_OAK);
    rbox(b, T(0, 0, hx - 0.07f), V3(hz - 0.08f, 0.04f, 0.011f), 0.002f, 1); rbox(b, T(0, 0, -hx + 0.07f), V3(hz - 0.08f, 0.04f, 0.011f), 0.002f, 1);
    add(m, b, T(cx, top - 0.074f, cz) * RY(rad(90)));
    Mesh b2; b2.set(M_WOOD, C_OAK); rbox(b2, T(0, 0, hz - 0.09f), V3(hx - 0.07f, 0.04f, 0.011f), 0.002f, 1); rbox(b2, T(0, 0, -hz + 0.09f), V3(hx - 0.07f, 0.04f, 0.011f), 0.002f, 1);
    add(m, b2, T(cx, top - 0.074f, cz)); }
  m.set(M_WOOD, C_OAK_V);
  for (int i = -1; i <= 1; i += 2) for (int j = -1; j <= 1; j += 2)
    rboxMM(m, V3(cx + i * (hx - 0.065f) - 0.027f, 0, cz + j * (hz - 0.085f) - 0.027f), V3(cx + i * (hx - 0.065f) + 0.027f, top - 0.03f, cz + j * (hz - 0.085f) + 0.027f), 0.005f, 2);
  // chairs: local +x faces the table
  Mesh ch; const u32 cc = RGBA(0.085f, 0.085f, 0.09f, 0.45f);
  ch.set(M_PLASTIC, cc);
  rbox(ch, T(0, 0.45f, 0), V3(0.205f, 0.011f, 0.205f), 0.010f, 3);
  for (int i = -1; i <= 1; i += 2) for (int j = -1; j <= 1; j += 2)
    cyl(ch, V3(i * 0.195f, 0, j * 0.195f), V3(i * 0.170f, 0.44f, j * 0.170f), 0.013f, 0.018f, 16, 0.002f);
  for (int j = -1; j <= 1; j += 2) {
    cyl(ch, V3(-0.182f, 0.21f, j * 0.182f), V3(0.182f, 0.21f, j * 0.182f), 0.008f, 0.008f, 10);
    cyl(ch, V3(-0.170f, 0.44f, j * 0.170f), V3(-0.215f, 0.88f, j * 0.165f), 0.016f, 0.012f, 16, 0.002f); // back posts
  }
  cyl(ch, V3(-0.180f, 0.30f, -0.18f), V3(-0.180f, 0.30f, 0.18f), 0.008f, 0.008f, 10);
  cyl(ch, V3(0.180f, 0.16f, -0.18f), V3(0.180f, 0.16f, 0.18f), 0.008f, 0.008f, 10);
  { Mesh b; b.set(M_PLASTIC, cc); rbox(b, T(0, 0, 0), V3(0.009f, 0.045f, 0.19f), 0.006f, 2); add(ch, b, T(-0.207f, 0.80f, 0) * RZ(rad(6))); add(ch, b, T(-0.196f, 0.66f, 0) * RZ(rad(6))); }
  add(m, ch, T(cx - 0.63f, 0, cz - 0.40f) * RY(rad(4)));
  add(m, ch, T(cx - 0.66f, 0, cz + 0.42f) * RY(rad(-7)));
  add(m, ch, T(cx + 0.63f, 0, cz - 0.38f) * RY(rad(183)));
  add(m, ch, T(cx + 0.70f, 0, cz + 0.45f) * RY(rad(172)));
}
static void buildBookshelf(Mesh& m) {
  Rng g(5);
  float x0 = 3.85f, x1 = 5.35f, z0 = 0.02f, z1 = 0.34f, H = 1.86f;
  m.set(M_WOOD, C_OAK_V);
  boxMM(m, V3(x0, 0, z0), V3(x0 + 0.02f, H, z1), 0.002f); boxMM(m, V3(x1 - 0.02f, 0, z0), V3(x1, H, z1), 0.002f);
  boxMM(m, V3(x0 + 0.745f, 0.09f, z0), V3(x0 + 0.765f, H, z1 - 0.005f), 0.002f);
  m.set(M_WOOD, C_OAK);
  boxMM(m, V3(x0, 0, z0), V3(x1, H, z0 + 0.008f), 0.001f);               // back panel
  boxMM(m, V3(x0 + 0.02f, 0, z1 - 0.035f), V3(x1 - 0.02f, 0.07f, z1 - 0.025f), 0.001f); // recessed plinth
  float sy[6] = {0.07f, 0.44f, 0.80f, 1.16f, 1.50f, H - 0.02f};
  for (int k = 0; k < 6; k++) boxMM(m, V3(x0 + 0.02f, sy[k], z0), V3(x1 - 0.02f, sy[k] + 0.02f, z1), 0.002f);
  const float pal[10][3] = {{0.12f, 0.17f, 0.30f}, {0.13f, 0.28f, 0.20f}, {0.45f, 0.12f, 0.12f}, {0.86f, 0.82f, 0.72f}, {0.80f, 0.58f, 0.18f},
                            {0.42f, 0.42f, 0.42f}, {0.07f, 0.07f, 0.08f}, {0.72f, 0.36f, 0.22f}, {0.25f, 0.40f, 0.50f}, {0.60f, 0.50f, 0.40f}};
  for (int k = 0; k < 5; k++) for (int bay = 0; bay < 2; bay++) {
    float a = bay ? x0 + 0.77f : x0 + 0.025f, b = bay ? x1 - 0.025f : x0 + 0.74f; float y = sy[k] + 0.02f; float x = a + g.r(0.0f, 0.02f);
    float maxH = sy[k + 1] - y - 0.03f;
    if (k == 0 && bay == 0) { // a storage basket on the bottom shelf
      m.set(M_FABRIC, RGBA(0.62f, 0.52f, 0.38f, 0.6f)); rboxMM(m, V3(a + 0.03f, y, z0 + 0.03f), V3(a + 0.40f, y + 0.25f, z1 - 0.02f), 0.03f, 3); x = a + 0.44f; }
    float stop = b - g.r(0.0f, 0.28f);
    while (true) {
      float th = g.r(0.016f, 0.046f), bh = std::min(maxH, g.r(0.19f, 0.30f)), bd = g.r(0.15f, 0.23f);
      if (x + th > stop) break;
      const float* c = pal[g.i(10)]; float v = g.r(0.8f, 1.1f);
      m.set(M_BOOK, RGBA(c[0] * v, c[1] * v, c[2] * v, g.f()));
      if (g.f() < 0.07f && x + bh + 0.02f < stop) { // a book lying flat
        Mesh bk; bk.set(m.mat, m.col); rbox(bk, T(0, 0, 0), V3(bh * 0.5f, th * 0.5f, bd * 0.5f), 0.002f, 1);
        add(m, bk, T(x + bh * 0.5f, y + th * 0.5f, z1 - 0.012f - bd * 0.5f)); x += bh + 0.012f; continue; }
      float tilt = g.f() < 0.12f ? rad(g.r(4, 9)) : 0;
      Mesh bk; bk.set(m.mat, m.col); rbox(bk, T(0, bh * 0.5f, 0), V3(th * 0.5f, bh * 0.5f, bd * 0.5f), 0.0018f, 1);
      add(m, bk, T(x + th * 0.5f + (tilt > 0 ? bh * 0.5f * sinf(tilt) : 0), y, z1 - 0.012f - bd * 0.5f - g.r(0, 0.012f)) * RZ(-tilt));
      x += th + (tilt > 0 ? bh * sinf(tilt) : 0) + g.r(0.0003f, 0.0012f);
    }
  }
}
static void buildSideboard(Mesh& m) {
  float x0 = 5.79f, x1 = 6.185f, z0 = 1.50f, z1 = 3.10f, y0 = 0.17f, y1 = 0.74f;
  m.set(M_WOOD, C_WALNUT);
  { Mesh b; b.set(M_WOOD, C_WALNUT); rbox(b, T(0, 0, 0), V3((z1 - z0) * 0.5f, (y1 - y0) * 0.5f, (x1 - x0) * 0.5f), 0.006f, 2); add(m, b, T((x0 + x1) * 0.5f, (y0 + y1) * 0.5f, (z0 + z1) * 0.5f) * RY(rad(90))); }
  for (int k = 0; k < 4; k++) { float a = z0 + 0.012f + k * (z1 - z0 - 0.024f) / 4, b = a + (z1 - z0 - 0.024f) / 4 - 0.004f;
    Mesh d; d.set(M_WOOD, C_WALNUT_V); rbox(d, T(0, 0, 0), V3(0.006f, (y1 - y0) * 0.5f - 0.014f, (b - a) * 0.5f), 0.003f, 2); add(m, d, T(x0 - 0.004f, (y0 + y1) * 0.5f, (a + b) * 0.5f));
    m.set(M_METAL, RGBA(0.80f, 0.66f, 0.38f, 0.3f)); cyl(m, V3(x0 - 0.010f, (y0 + y1) * 0.5f + 0.06f, k % 2 ? a + 0.03f : b - 0.03f), V3(x0 - 0.030f, (y0 + y1) * 0.5f + 0.06f, k % 2 ? a + 0.03f : b - 0.03f), 0.007f, 0.010f, 14); }
  m.set(M_WOOD, C_WALNUT_V);
  for (int i = 0; i < 2; i++) for (int j = 0; j < 2; j++)
    cyl(m, V3(i ? x1 - 0.045f : x0 + 0.035f, 0, j ? z1 - 0.07f : z0 + 0.07f), V3(i ? x1 - 0.05f : x0 + 0.05f, y0, j ? z1 - 0.09f : z0 + 0.09f), 0.012f, 0.021f, 18, 0.002f);
  // on top: vase and a small lamp
  m.set(M_CERAMIC, RGBA(0.20f, 0.33f, 0.36f, 0.1f));
  lathe(m, T(5.98f, y1, 1.85f), Profile{{0, 0}, {0.05f, 0}, {0.085f, 0.07f}, {0.095f, 0.15f}, {0.07f, 0.26f}, {0.035f, 0.33f}, {0.04f, 0.37f}, {0.032f, 0.37f}, {0.028f, 0.33f}, {0, 0.30f}}, 36);
  m.set(M_METAL, RGBA(0.80f, 0.66f, 0.38f, 0.3f));
  lathe(m, T(5.98f, y1, 2.75f), Profile{{0, 0}, {0.07f, 0}, {0.07f, 0.012f}, {0.012f, 0.02f}, {0.010f, 0.30f}, {0, 0.30f}}, 28);
  m.set(M_CURTAIN, RGBA(0.90f, 0.86f, 0.76f));
  lathe(m, T(5.98f, y1 + 0.26f, 2.75f), Profile{{0.13f, 0}, {0.095f, 0.19f}, {0.093f, 0.19f}, {0.128f, 0}}, 36);
  // framed print above
  m.set(M_PLASTIC, RGBA(0.06f, 0.06f, 0.06f, 0.5f));
  boxMM(m, V3(6.175f, 1.10f, 1.88f), V3(6.2f, 1.96f, 2.56f), 0.004f, 2);
  m.set(M_ART, RGBA(1, 1, 1));
  { u32 a = m.v(V3(6.174f, 1.13f, 1.91f), V3(-1, 0, 0), V3(0, 0, 0.31f)), b = m.v(V3(6.174f, 1.13f, 2.53f), V3(-1, 0, 0), V3(1, 0, 0.31f)), c = m.v(V3(6.174f, 1.93f, 2.53f), V3(-1, 0, 0), V3(1, 1, 0.31f)), d = m.v(V3(6.174f, 1.93f, 1.91f), V3(-1, 0, 0), V3(0, 1, 0.31f)); m.quad(a, b, c, d); }
}
static void buildLampAndArt(Mesh& m) {
  // floor lamp right of the sofa
  float lx = 3.42f, lz = 0.25f;
  m.set(M_METAL, RGBA(0.10f, 0.10f, 0.10f, 0.45f));
  lathe(m, T(lx, 0, lz), Profile{{0, 0}, {0.125f, 0}, {0.13f, 0.004f}, {0.13f, 0.014f}, {0.125f, 0.018f}, {0.02f, 0.024f}, {0.011f, 0.05f}, {0.010f, 1.50f}, {0, 1.50f}}, 40);
  m.set(M_CURTAIN, RGBA(0.92f, 0.88f, 0.78f));
  lathe(m, T(lx, 1.40f, lz), Profile{{0.20f, 0}, {0.15f, 0.27f}, {0.148f, 0.27f}, {0.198f, 0}}, 40);
  m.set(M_RUBBER, RGBA(0.05f, 0.05f, 0.05f));
  tubeC(m, spline({V3(lx + 0.12f, 0.0035f, lz - 0.03f), V3(lx + 0.20f, 0.0035f, lz - 0.12f), V3(lx + 0.27f, 0.0035f, lz - 0.20f), V3(lx + 0.295f, 0.02f, lz - 0.225f), V3(lx + 0.30f, 0.14f, lz - 0.232f), V3(lx + 0.30f, 0.25f, lz - 0.236f)}, 6), 0.0032f, 8);
  // framed print above the sofa (north wall)
  m.set(M_WOOD, C_OAK);
  boxMM(m, V3(1.42f, 1.18f, 0), V3(2.58f, 1.98f, 0.022f), 0.004f, 2);
  m.set(M_ART, RGBA(1, 1, 1));
  { u32 a = m.v(V3(1.46f, 1.22f, 0.0225f), V3(0, 0, 1), V3(0, 0, 0.77f)), b = m.v(V3(2.54f, 1.22f, 0.0225f), V3(0, 0, 1), V3(1.4f, 0, 0.77f)), c = m.v(V3(2.54f, 1.94f, 0.0225f), V3(0, 0, 1), V3(1.4f, 1, 0.77f)), d = m.v(V3(1.46f, 1.94f, 0.0225f), V3(0, 0, 1), V3(0, 1, 0.77f)); m.quad(a, b, c, d); }
  // charging dock on the west wall
  m.set(M_PLASTIC, RGBA(0.05f, 0.05f, 0.055f, 0.35f));
  boxMM(m, V3(0.014f, 0, 2.87f), V3(0.075f, 0.105f, 3.03f), 0.012f, 3);
  boxMM(m, V3(0.014f, 0, 2.88f), V3(0.20f, 0.010f, 3.02f), 0.004f, 2);
  m.set(M_EMIT, RGBA(0.2f, 1.0f, 0.4f, 0.12f));
  boxMM(m, V3(0.074f, 0.085f, 2.946f), V3(0.077f, 0.089f, 2.954f), 0.001f);
}

// ---------------------------------------------------------------- curtains
static void curtain(Mesh& m, float x0, float x1, float z, int folds, float phase) {
  const int NU = 14 * folds, NV = 30; float yTop = 2.50f, yBot = 0.018f;
  u32 base = (u32)m.p.size(); std::vector<V3> P((NU + 1) * (NV + 1));
  for (int j = 0; j <= NV; j++) for (int i = 0; i <= NU; i++) {
    float a = (float)i / NU, b = (float)j / NV; float y = lerpf(yTop, yBot, b);
    float amp = 0.030f * (0.75f + 0.25f * b) * (1.0f + 0.25f * sinf(a * 7.1f + phase * 3));
    float zz = z + amp * sinf(a * folds * 2 * PI + phase + 0.5f * sinf(b * 2.2f + a * 5)) + 0.004f * sinf(b * 9 + a * 30);
    P[j * (NU + 1) + i] = V3(lerpf(x0, x1, a) + 0.006f * sinf(b * 3 + phase), y, zz);
  }
  for (int j = 0; j <= NV; j++) for (int i = 0; i <= NU; i++) {
    V3 du = P[j * (NU + 1) + std::min(i + 1, NU)] - P[j * (NU + 1) + std::max(i - 1, 0)], dv = P[std::min(j + 1, NV) * (NU + 1) + i] - P[std::max(j - 1, 0) * (NU + 1) + i];
    float s = (float)i / NU * (x1 - x0) * 3.2f; // cloth-space coordinate (the fabric is ~3x wider than the gathered panel)
    m.v(P[j * (NU + 1) + i], norm(cross(dv, du)), V3(s, P[j * (NU + 1) + i].y, 0));
  }
  for (int j = 0; j < NV; j++) for (int i = 0; i < NU; i++) { u32 a = base + j * (NU + 1) + i; m.quad(a, a + 1, a + NU + 2, a + NU + 1); }
}
static void buildCurtains(Mesh& m, Scene& sc) {
  float z = ROOM_Z - 0.085f;
  m.set(M_CURTAIN, RGBA(0.90f, 0.87f, 0.80f));
  curtain(m, 0.70f, 1.05f, z, 4, 0.3f); curtain(m, 2.07f, 2.34f, z, 3, 1.7f);
  curtain(m, 3.08f, 3.44f, z, 4, 2.9f); curtain(m, 4.47f, 4.82f, z, 4, 4.1f);
  m.set(M_METAL, RGBA(0.12f, 0.12f, 0.12f, 0.4f));
  cyl(m, V3(0.62f, 2.52f, z), V3(2.42f, 2.52f, z), 0.011f, 0.011f, 12); cyl(m, V3(3.0f, 2.52f, z), V3(4.9f, 2.52f, z), 0.011f, 0.011f, 12);
}

// ---------------------------------------------------------------- rubber plant in a pot by the window
static void leaf(Mesh& m, V3 base, V3 dir, V3 upHint, float L, float Wd, float droop, float fold, float seed) {
  const int NU = 12, NV = 6; u32 b0 = (u32)m.p.size(); std::vector<V3> P((NU + 1) * (NV + 1));
  dir = norm(dir); V3 side = norm(cross(dir, upHint)); V3 nup = norm(cross(side, dir));
  for (int i = 0; i <= NU; i++) {
    float u = (float)i / NU; float w = Wd * 0.5f * powf(sinf(PI * powf(u, 0.72f)), 0.75f) * (u > 0.999f ? 0.0f : 1.0f) + 0.0015f;
    V3 c = base + dir * (u * L) - nup * (droop * u * u * L) + side * (0.03f * L * sinf(u * 3 + seed * 20));
    for (int j = 0; j <= NV; j++) { float v = -1.0f + 2.0f * j / NV; P[i * (NV + 1) + j] = c + side * (v * w) + nup * (fabsf(v) * w * fold + 0.004f * sinf(u * 16 + v * 5 + seed * 40) * w / (Wd * 0.5f + 1e-5f)); }
  }
  for (int i = 0; i <= NU; i++) for (int j = 0; j <= NV; j++) {
    V3 du = P[std::min(i + 1, NU) * (NV + 1) + j] - P[std::max(i - 1, 0) * (NV + 1) + j], dv = P[i * (NV + 1) + std::min(j + 1, NV)] - P[i * (NV + 1) + std::max(j - 1, 0)];
    m.v(P[i * (NV + 1) + j], norm(cross(dv, du)), V3((float)i / NU, -1.0f + 2.0f * j / NV, seed));
  }
  for (int i = 0; i < NU; i++) for (int j = 0; j < NV; j++) { u32 a = b0 + i * (NV + 1) + j; m.quad(a, a + 1, a + NV + 2, a + NV + 1); }
}
static void buildPlant(Mesh& m) {
  Rng g(41); V3 c(2.30f, 0, 4.00f);
  m.set(M_CERAMIC, RGBA(0.80f, 0.78f, 0.74f, 0.9f));
  lathe(m, T(c), Profile{{0, 0}, {0.135f, 0}, {0.145f, 0.008f}, {0.185f, 0.33f}, {0.195f, 0.345f}, {0.195f, 0.37f}, {0.180f, 0.37f}, {0.172f, 0.33f}, {0.170f, 0.31f}, {0, 0.31f}}, 48);
  m.set(M_MATTE, RGBA(0.13f, 0.09f, 0.06f));
  lathe(m, T(c + V3(0, 0.315f, 0)), Profile{{0.171f, 0}, {0.10f, 0.012f}, {0, 0.016f}}, 32);
  for (int s = 0; s < 3; s++) {
    float a0 = s * 2.1f + 0.4f; V3 b = c + V3(0.04f * cosf(a0), 0.32f, 0.04f * sinf(a0)); float H = g.r(0.85f, 1.25f);
    std::vector<V3> ctl; for (int k = 0; k <= 5; k++) { float u = k / 5.0f; ctl.push_back(b + V3(cosf(a0) * 0.16f * u * u + 0.03f * sinf(u * 5 + s), H * u, sinf(a0) * 0.16f * u * u + 0.03f * cosf(u * 4 + s))); }
    std::vector<V3> st = spline(ctl, 6); std::vector<float> rr(st.size());
    for (size_t k = 0; k < st.size(); k++) rr[k] = lerpf(0.011f, 0.004f, (float)k / (st.size() - 1));
    m.set(M_WOOD, RGBA(0.30f, 0.24f, 0.16f, 1.0f)); tube(m, st, rr, 8);
    int nl = 11 + g.i(4);
    for (int k = 0; k < nl; k++) {
      float u = 0.16f + 0.84f * (k + g.f() * 0.5f) / nl; size_t id = std::min(st.size() - 2, (size_t)(u * (st.size() - 1)));
      V3 p = st[id], tng = norm(st[id + 1] - st[id]); float ang = k * 2.4f + s * 1.1f + g.r(-0.3f, 0.3f);
      V3 out = norm(V3(cosf(ang), 0, sinf(ang))); float el = lerpf(rad(20), rad(62), u) + g.r(-0.15f, 0.15f);
      V3 d = norm(out * cosf(el) + tng * sinf(el)); float L = g.r(0.17f, 0.26f) * (1.0f - 0.3f * u * u);
      // petiole
      m.set(M_LEAF, RGBA(0.20f, 0.30f, 0.10f)); tubeC(m, {p, p + d * 0.035f}, 0.0022f, 5, false);
      float v = g.r(0.85f, 1.15f);
      m.set(M_LEAF, RGBA(0.075f * v, 0.20f * v, 0.065f * v));
      leaf(m, p + d * 0.033f, d, V3(0, 1, 0), L, L * g.r(0.46f, 0.56f), g.r(0.10f, 0.32f), g.r(0.10f, 0.22f), g.f());
    }
  }
}

// ---------------------------------------------------------------- toys and the small stuff on the floor
static void prism(Mesh& m, const M34& xf, float w, float h, float d) { // triangular block
  Mesh b; b.set(m.mat, m.col);
  V3 a(-w / 2, 0, -d / 2), bb(w / 2, 0, -d / 2), c(0, h, -d / 2), a2(-w / 2, 0, d / 2), b2(w / 2, 0, d / 2), c2(0, h, d / 2);
  quad(b, a, bb, b2, a2); quad(b, bb, c, c2, b2); quad(b, c, a, a2, c2);
  { V3 n(0, 0, -1); u32 i0 = b.v(a, n, a), i1 = b.v(c, n, c), i2 = b.v(bb, n, bb); b.tri(i0, i1, i2); }
  { V3 n(0, 0, 1); u32 i0 = b.v(a2, n, a2), i1 = b.v(b2, n, b2), i2 = b.v(c2, n, c2); b.tri(i0, i1, i2); }
  add(m, b, xf);
}
static void legoBrick(Mesh& m, const M34& xf, int nx, int nz, u32 col) {
  Mesh b; b.set(M_PLASTIC, col); float P = 0.008f;
  rbox(b, T(0, 0.0048f, 0), V3(nx * P * 0.5f - 0.0001f, 0.0048f, nz * P * 0.5f - 0.0001f), 0.0004f, 1);
  for (int i = 0; i < nx; i++) for (int j = 0; j < nz; j++) {
    V3 c((i - (nx - 1) * 0.5f) * P, 0.0096f, (j - (nz - 1) * 0.5f) * P);
    lathe(b, T(c), Profile{{0.0024f, -0.0002f}, {0.0024f, 0.0015f}, {0.0022f, 0.0017f}, {0, 0.0017f}}, 14);
  }
  add(m, b, xf);
}
static void dustBunny(Mesh& m, V3 c, float R, Rng& g) {
  m.set(M_MATTE, RGBA(0.52f, 0.50f, 0.47f));
  for (int h = 0; h < 170; h++) {
    V3 p = c + V3(g.g() * R * 0.45f, fabsf(g.g()) * R * 0.22f + 0.001f, g.g() * R * 0.45f); V3 d = norm(V3(g.g(), g.g() * 0.5f, g.g()));
    float w = g.r(0.00018f, 0.00045f); V3 prev = p; V3 sd = norm(cross(d, V3(0.3f, 1, 0.2f)));
    for (int k = 0; k < 6; k++) {
      d = norm(d + V3(g.g(), g.g() * 0.6f, g.g()) * 0.75f); V3 q = prev + d * (R * 0.32f); q.y = std::max(q.y, 0.0006f);
      V3 n = norm(cross(q - prev, sd)); u32 a = m.v(prev - sd * w, n, prev), b = m.v(prev + sd * w, n, prev), cc = m.v(q + sd * w, n, q), dd = m.v(q - sd * w, n, q);
      m.quad(a, b, cc, dd); prev = q;
    }
  }
  for (int k = 0; k < 5; k++) { m.set(M_MATTE, RGBA(0.42f, 0.40f, 0.38f)); ellipsoid(m, T(c + V3(g.g() * R * 0.3f, R * 0.12f, g.g() * R * 0.3f)), V3(R * g.r(0.12f, 0.22f), R * 0.10f, R * g.r(0.12f, 0.22f)), 8, 5); }
}
static void buildToys(Mesh& m, Scene& sc) {
  Rng g(97);
  const u32 RED = RGBA(0.78f, 0.15f, 0.11f, 0.5f), BLUE = RGBA(0.13f, 0.33f, 0.60f, 0.5f), YEL = RGBA(0.92f, 0.71f, 0.13f, 0.5f), GRN = RGBA(0.24f, 0.52f, 0.26f, 0.5f), NAT = RGBA(0.83f, 0.68f, 0.47f, 0.0f);
  const float B = 0.045f;
  auto cube = [&](V3 c, float ry, u32 col, V3 h) { Mesh b; b.set(col == NAT ? M_WOOD : M_PLASTIC, col); rbox(b, T(0, 0, 0), h, 0.0025f, 2); add(m, b, T(c) * RY(ry)); };
  // little tower
  V3 t0(1.86f, 0, 3.42f);
  cube(t0 + V3(-0.030f, B / 2, 0), rad(8), RED, V3(B / 2, B / 2, B / 2)); cube(t0 + V3(0.034f, B / 2, 0.004f), rad(-5), NAT, V3(B / 2, B / 2, B / 2));
  cube(t0 + V3(0.002f, B + 0.0112f, 0.002f), rad(3), BLUE, V3(B * 1.25f, 0.011f, B / 2));
  cube(t0 + V3(0.0f, B + 0.0225f + B / 2, 0), rad(24), YEL, V3(B / 2, B / 2, B / 2));
  m.set(M_PLASTIC, GRN); prism(m, T(t0 + V3(0, 2 * B + 0.0225f, 0)) * RY(rad(24)), B, B * 0.8f, B);
  // scattered blocks
  cube(V3(1.62f, B / 2, 3.60f), rad(31), GRN, V3(B / 2, B / 2, B / 2)); cube(V3(2.02f, B / 2, 3.24f), rad(-18), YEL, V3(B / 2, B / 2, B / 2));
  cube(V3(1.70f, 0.011f, 3.20f), rad(64), RED, V3(B * 1.25f, 0.011f, B / 2));
  m.set(M_PLASTIC, BLUE); { Mesh b; b.set(M_PLASTIC, BLUE); lathe(b, T(0, 0, 0), roundedCylProfile(B / 2, B / 2, B, 0.0025f), 28, 50); add(m, b, T(2.10f, B / 2, 3.52f) * RY(rad(40)) * RZ(rad(90)) * T(0, -B / 2, 0)); }
  m.set(M_WOOD, NAT); { Mesh b; b.set(M_PLASTIC, RED); prism(b, T(0, 0, 0), B, B * 0.8f, B); add(m, b, T(1.52f, 0, 3.34f) * RY(rad(-40))); }
  // stacking rings
  V3 sr(2.02f, 0, 3.80f);
  m.set(M_PLASTIC, RGBA(0.92f, 0.90f, 0.84f, 0.25f));
  lathe(m, T(sr), Profile{{0, 0}, {0.058f, 0}, {0.062f, 0.004f}, {0.058f, 0.012f}, {0.014f, 0.016f}, {0.008f, 0.150f}, {0.006f, 0.154f}, {0, 0.155f}}, 32);
  const float rc[5][3] = {{0.80f, 0.16f, 0.14f}, {0.93f, 0.50f, 0.12f}, {0.94f, 0.76f, 0.15f}, {0.25f, 0.58f, 0.30f}, {0.16f, 0.40f, 0.70f}};
  float yy = 0.016f;
  for (int k = 0; k < 5; k++) { float R = 0.046f - k * 0.0062f, r = 0.0125f - k * 0.0009f; m.set(M_PLASTIC, RGBA(rc[k][0], rc[k][1], rc[k][2], 0.15f)); torus(m, T(sr + V3(g.r(-0.002f, 0.002f), yy + r, g.r(-0.002f, 0.002f))), R, r, 40, 14); yy += 2 * r - 0.001f; }
  // ball
  m.set(M_PLASTIC, RGBA(0.86f, 0.25f, 0.20f, 0.35f)); ellipsoid(m, T(1.50f, 0.055f, 3.78f), V3(0.055f, 0.055f, 0.055f), 48, 24);
  // wooden train: engine + wagon
  { Mesh tr; auto wheels = [&](float x) { for (int s = -1; s <= 1; s += 2) { tr.set(M_PLASTIC, RGBA(0.07f, 0.07f, 0.07f, 0.5f)); cyl(tr, V3(x, 0.014f, s * 0.021f), V3(x, 0.014f, s * 0.031f), 0.014f, 0.014f, 20, 0.002f); } };
    tr.set(M_WOOD, NAT); rbox(tr, T(0, 0.022f, 0), V3(0.05f, 0.008f, 0.02f), 0.002f, 2);
    tr.set(M_PLASTIC, RED); { Mesh b; b.set(M_PLASTIC, RED); lathe(b, T(0, 0, 0), roundedCylProfile(0.016f, 0.016f, 0.055f, 0.002f), 24, 50); add(tr, b, T(-0.005f, 0.046f, 0) * RZ(rad(-90))); }
    tr.set(M_PLASTIC, BLUE); rbox(tr, T(-0.030f, 0.052f, 0), V3(0.018f, 0.022f, 0.02f), 0.002f, 2);
    tr.set(M_PLASTIC, YEL); cyl(tr, V3(0.034f, 0.060f, 0), V3(0.034f, 0.078f, 0), 0.006f, 0.008f, 14, 0.001f);
    wheels(0.028f); wheels(-0.028f);
    tr.set(M_WOOD, NAT); rbox(tr, T(-0.115f, 0.022f, 0), V3(0.042f, 0.008f, 0.02f), 0.002f, 2);
    tr.set(M_PLASTIC, GRN); rbox(tr, T(-0.128f, 0.045f, 0), V3(0.016f, 0.015f, 0.016f), 0.002f, 2);
    tr.set(M_PLASTIC, YEL); rbox(tr, T(-0.096f, 0.043f, 0), V3(0.013f, 0.013f, 0.016f), 0.002f, 2);
    wheels(-0.092f); wheels(-0.138f);
    tr.set(M_METAL, RGBA(0.6f, 0.6f, 0.6f, 0.4f)); cyl(tr, V3(-0.05f, 0.022f, 0), V3(-0.073f, 0.022f, 0), 0.0025f, 0.0025f, 8);
    add(m, tr, T(1.34f, 0, 3.30f) * RY(rad(-28))); }
  // LEGO: some on the open floor, some lost under the sofa
  legoBrick(m, T(1.93f, 0, 3.62f) * RY(rad(25)), 4, 2, RGBA(0.90f, 0.90f, 0.88f, 0.12f));
  legoBrick(m, T(2.50f, 0, 0.90f) * RY(rad(33)), 4, 2, RGBA(0.78f, 0.10f, 0.08f, 0.12f));
  legoBrick(m, T(2.74f, 0, 0.86f) * RY(rad(-20)), 2, 2, RGBA(0.94f, 0.75f, 0.08f, 0.12f));
  legoBrick(m, T(1.78f, 0, 0.44f) * RY(rad(70)), 4, 2, RGBA(0.08f, 0.30f, 0.68f, 0.12f));
  // under the sofa: sock, coin, hair tie, crayon, bouncy ball, dust
  m.set(M_FABRIC, RGBA(0.30f, 0.34f, 0.44f, 0.4f));
  sdfMesh(m, T(2.72f, 0, 0.29f) * RY(rad(20)), [](V3 p) {
    float d = sdCapsule(p, V3(-0.07f, 0.014f, 0), V3(0.03f, 0.015f, 0.004f), 0.021f, 0.024f);
    d = smin(d, sdCapsule(p, V3(0.03f, 0.015f, 0.004f), V3(0.065f, 0.016f, 0.05f), 0.024f, 0.021f), 0.02f);
    d += 0.0035f * sinf(p.x * 90) * sinf(p.z * 70 + 1);
    V3 q(p.x, p.y / 0.62f, p.z); (void)q;
    return std::max(d * 1.0f + (p.y > 0.020f ? (p.y - 0.020f) * 1.6f : 0.0f), -p.y); }, V3(-0.11f, -0.002f, -0.04f), V3(0.10f, 0.05f, 0.09f), 0.0016f);
  m.set(M_METAL, RGBA(0.85f, 0.62f, 0.32f, 0.35f)); lathe(m, T(2.36f, 0, 0.33f), Profile{{0, 0}, {0.0105f, 0}, {0.0112f, 0.0006f}, {0.0112f, 0.0014f}, {0.0105f, 0.002f}, {0.0095f, 0.0017f}, {0, 0.0017f}}, 28);
  m.set(M_RUBBER, RGBA(0.03f, 0.03f, 0.03f)); torus(m, T(2.93f, 0.002f, 0.27f), 0.021f, 0.002f, 28, 6);
  m.set(M_PLASTIC, RGBA(0.93f, 0.42f, 0.10f, 0.6f)); { Mesh b; b.set(m.mat, m.col); lathe(b, T(0, 0, 0), Profile{{0, 0}, {0.0045f, 0}, {0.0045f, 0.072f}, {0.0016f, 0.086f}, {0, 0.087f}}, 12); add(m, b, T(1.86f, 0.0045f, 0.33f) * RY(rad(28)) * RZ(rad(90))); }
  m.set(M_PLASTIC, RGBA(0.20f, 0.72f, 0.50f, 0.2f)); ellipsoid(m, T(2.22f, 0.016f, 0.22f), V3(0.016f, 0.016f, 0.016f), 28, 14);
  V3 db[9] = {V3(1.30f, 0, 0.12f), V3(1.95f, 0, 0.11f), V3(2.48f, 0, 0.13f), V3(2.97f, 0, 0.14f), V3(1.52f, 0, 0.50f), V3(2.60f, 0, 0.36f), V3(1.02f, 0, 0.60f), V3(3.02f, 0, 0.42f), V3(2.12f, 0, 0.19f)};
  for (int k = 0; k < 9; k++) dustBunny(m, db[k], g.r(0.016f, 0.034f), g);
  // under the dining table: breakfast cereal and crumbs
  for (int k = 0; k < 15; k++) {
    V3 c(TABLE_X + g.r(-0.26f, 0.26f), 0.0027f, g.r(1.35f, 2.95f)); if (k < 5) c.x = TABLE_X + g.r(-0.09f, 0.09f);
    float v = g.r(0.85f, 1.1f); m.set(M_MATTE, RGBA(0.82f * v, 0.60f * v, 0.30f * v));
    Mesh b; b.set(m.mat, m.col); torus(b, T(0, 0, 0), 0.0058f, 0.0027f, 18, 8); add(m, b, T(c) * RY(g.r(0, 6.28f)) * RX(g.f() < 0.25f ? rad(g.r(8, 20)) : 0));
    sc.cereal.push_back(c);
  }
  for (int k = 0; k < 70; k++) { V3 c(TABLE_X + g.r(-0.38f, 0.38f), 0.0008f, g.r(1.3f, 3.1f)); float s = g.r(0.0006f, 0.0022f); float v = g.r(0.7f, 1.1f);
    m.set(M_MATTE, RGBA(0.78f * v, 0.62f * v, 0.40f * v)); ellipsoid(m, T(c), V3(s, s * 0.6f, s * g.r(0.6f, 1.3f)), 7, 4); }
  // a few crumbs and specks elsewhere so the floor is not sterile
  for (int k = 0; k < 120; k++) { V3 c(g.r(0.2f, 6.0f), 0.0004f, g.r(0.2f, 4.2f)); float s = g.r(0.0004f, 0.0012f); float v = g.r(0.3f, 0.9f);
    m.set(M_MATTE, RGBA(0.6f * v, 0.55f * v, 0.48f * v)); ellipsoid(m, T(c), V3(s, s * 0.5f, s), 6, 3); }
}

// ---------------------------------------------------------------- balcony and what is outside the windows
static void buildOutside(Mesh& m) {
  float z0 = ROOM_Z + WALL_T;
  m.set(M_CONCRETE, RGBA(0.62f, 0.60f, 0.57f));
  boxMM(m, V3(0.2f, -0.25f, z0), V3(6.0f, 0.03f, z0 + 1.35f), 0.004f);
  m.set(M_PLASTIC, RGBA(0.13f, 0.14f, 0.15f, 0.5f));
  float zr = z0 + 1.30f;
  cyl(m, V3(0.2f, 1.06f, zr), V3(6.0f, 1.06f, zr), 0.022f, 0.022f, 12); cyl(m, V3(0.2f, 0.09f, zr), V3(6.0f, 0.09f, zr), 0.012f, 0.012f, 8);
  for (float x = 0.25f; x < 6.0f; x += 0.115f) cyl(m, V3(x, 0.09f, zr), V3(x, 1.06f, zr), 0.0075f, 0.0075f, 8);
  // planter with something green on the balcony
  m.set(M_CERAMIC, RGBA(0.62f, 0.34f, 0.22f, 0.9f)); boxMM(m, V3(4.7f, 0.03f, z0 + 0.95f), V3(5.7f, 0.30f, z0 + 1.22f), 0.012f, 2);
  // far ground, neighbours, trees
  m.set(M_MATTE, RGBA(0.30f, 0.34f, 0.26f));
  quad(m, V3(-200, -9, -50), V3(-200, -9, 400), V3(300, -9, 400), V3(300, -9, -50));
  m.set(M_FACADE, RGBA(0.72f, 0.63f, 0.52f));
  { u32 a = m.v(V3(-34, -9, 44), V3(0, 0, -1), V3(0, 0, 0)), b = m.v(V3(-34, 13, 44), V3(0, 0, -1), V3(0, 22, 0)), c = m.v(V3(22, 13, 44), V3(0, 0, -1), V3(56, 22, 0)), d = m.v(V3(22, -9, 44), V3(0, 0, -1), V3(56, 0, 0)); m.quad(a, b, c, d); }
  m.set(M_FACADE, RGBA(0.80f, 0.78f, 0.74f));
  { u32 a = m.v(V3(26, -9, 60), V3(0, 0, -1), V3(0, 0, 0)), b = m.v(V3(26, 19, 60), V3(0, 0, -1), V3(0, 28, 0)), c = m.v(V3(70, 19, 60), V3(0, 0, -1), V3(44, 28, 0)), d = m.v(V3(70, -9, 60), V3(0, 0, -1), V3(44, 0, 0)); m.quad(a, b, c, d); }
  Rng g(3);
  V3 tc[7] = {V3(-6, 1.0f, 17), V3(1.5f, 2.2f, 21), V3(8, 0.5f, 18), V3(13, 2.5f, 24), V3(-13, 2.0f, 23), V3(5, 3.0f, 30), V3(18, 1.5f, 19)};
  for (int k = 0; k < 7; k++) {
    Mesh b; float v = g.r(0.8f, 1.2f); b.set(M_FOLIAGE, RGBA(0.13f * v, 0.21f * v, 0.085f * v));
    ellipsoid(b, T(0, 0, 0), V3(1, 1, 1), 40, 20);
    for (size_t i = 0; i < b.p.size(); i++) { V3 p = b.p[i]; float d = 0.62f + 0.75f * fbm(p * 1.7f + V3(k * 7.0f, 0, 0), 4); b.p[i] = p * d; b.t[i] = b.p[i] * 3.0f + V3(k * 5.0f, 0, 0); }
    float R = g.r(3.0f, 4.6f); add(m, b, T(tc[k]) * S(R, R * g.r(0.8f, 1.0f), R));
  }
}

// ---------------------------------------------------------------- the robot (robot-local: +x forward, +y up, +z right)
static void buildRobot(Scene& sc) {
  Mesh deck;
  deck.set(M_ROBOT, RGBA(0.035f, 0.035f, 0.04f, 0.2f));
  lathe(deck, T(0, 0, 0), Profile{{0.150f, ROBOT_TOP - 0.0004f}, {0.1257f, ROBOT_TOP}, {0.1246f, ROBOT_TOP - 0.0011f}, {0.1235f, ROBOT_TOP}, {0, ROBOT_TOP + 0.0006f}}, 160, 30);
  deck.set(M_ROBOT, RGBA(0.05f, 0.05f, 0.055f, 0.9f));
  { Profile p; p.push_back({0.150f, ROBOT_TOP - 0.0004f}); p.push_back({0.1515f, ROBOT_TOP - 0.0016f}); p.push_back({0.1530f, ROBOT_TOP - 0.0006f});
    for (int i = 0; i <= 8; i++) { float a = rad(90) * (1 - i / 8.0f); p.push_back({0.158f + 0.012f * cosf(a), ROBOT_TOP - 0.0126f + 0.012f * sinf(a)}); }
    p.push_back({0.170f, 0.028f}); p.push_back({0.166f, 0.016f}); p.push_back({0.150f, 0.012f}); p.push_back({0, 0.012f});
    std::reverse(p.begin(), p.end()); lathe(deck, T(0, 0, 0), p, 160, 30); }
  deck.set(M_EMIT, RGBA(0.25f, 1.0f, 0.45f, 0.10f));
  lathe(deck, T(0.139f, ROBOT_TOP + 0.0002f, 0.0f), Profile{{0.0026f, 0}, {0.0022f, 0.0006f}, {0, 0.0006f}}, 12);
  sc.robot = deck;
  // lidar turret and the camera housing: only ever seen as shadows and reflections
  sc.robot.set(M_ROBOT, RGBA(0.04f, 0.04f, 0.045f, 0.9f));
  cyl(sc.robot, V3(-0.02f, ROBOT_TOP, 0), V3(-0.02f, ROBOT_TOP + 0.024f, 0), 0.046f, 0.045f, 28, 0.003f);
  rbox(sc.robot, T(CAM_FWD - 0.012f, ROBOT_TOP + 0.022f, 0), V3(0.014f, 0.022f, 0.031f), 0.004f, 2);
  // camera-space copy of the deck
  sc.shell = deck;
  for (size_t i = 0; i < sc.shell.p.size(); i++) { sc.shell.p[i] = robotToCam(deck.p[i]); sc.shell.n[i] = robotDirToCam(deck.n[i]); }
}

// ---------------------------------------------------------------- toy T-Rex (local: +x forward, +y up; built in "dino units", standing height ~1)
static const float DINO_S = 0.15f;
struct DinoParts { float body, spikes, claws; };
static DinoParts dinoSDF(V3 p) {
  DinoParts o; const float K = 0.07f;
  float d = sdCapsule(p, V3(-0.07f, 0.41f, 0), V3(0.085f, 0.64f, 0), 0.215f, 0.175f);                      // torso
  d = smin(d, sdCapsule(p, V3(0.085f, 0.64f, 0), V3(0.15f, 0.83f, 0), 0.135f, 0.12f), K);                  // neck
  d = smin(d, sdEllipsoid(p, V3(0.19f, 0.895f, 0), V3(0.185f, 0.125f, 0.135f)), 0.05f);                    // skull
  d = smin(d, sdCapsule(p, V3(0.22f, 0.885f, 0), V3(0.345f, 0.865f, 0), 0.105f, 0.082f), 0.04f);           // snout
  d = smin(d, sdCapsule(p, V3(0.11f, 0.795f, 0), V3(0.325f, 0.762f, 0), 0.074f, 0.048f), 0.016f);          // lower jaw (small blend keeps a mouth line)
  for (int s = -1; s <= 1; s += 2) {
    float z = (float)s;
    d = smin(d, sdSphere(p, V3(0.165f, 0.985f, z * 0.082f), 0.05f), 0.03f);                                // brow
    d = smin(d, sdSphere(p, V3(0.395f, 0.895f, z * 0.036f), 0.022f), 0.02f);                               // nostril bump
    d = smin(d, sdCapsule(p, V3(-0.03f, 0.43f, z * 0.15f), V3(0.075f, 0.245f, z * 0.175f), 0.128f, 0.092f), K);   // thigh
    d = smin(d, sdCapsule(p, V3(0.075f, 0.245f, z * 0.175f), V3(0.00f, 0.10f, z * 0.175f), 0.080f, 0.060f), 0.04f); // shin
    d = smin(d, sdCapsule(p, V3(0.00f, 0.075f, z * 0.175f), V3(0.20f, 0.05f, z * 0.178f), 0.068f, 0.052f), 0.04f);  // foot
    d = smin(d, sdCapsule(p, V3(0.13f, 0.60f, z * 0.15f), V3(0.215f, 0.525f, z * 0.165f), 0.046f, 0.036f), 0.03f);  // upper arm
    d = smin(d, sdCapsule(p, V3(0.215f, 0.525f, z * 0.165f), V3(0.285f, 0.555f, z * 0.150f), 0.034f, 0.027f), 0.02f); // forearm
  }
  d = smin(d, sdCapsule(p, V3(-0.10f, 0.40f, 0), V3(-0.37f, 0.215f, 0.03f), 0.18f, 0.108f), K);            // tail
  d = smin(d, sdCapsule(p, V3(-0.37f, 0.215f, 0.03f), V3(-0.60f, 0.085f, 0.075f), 0.108f, 0.052f), 0.05f);
  d = smin(d, sdCapsule(p, V3(-0.60f, 0.085f, 0.075f), V3(-0.75f, 0.04f, 0.13f), 0.052f, 0.028f), 0.03f);
  // claws / toes
  float c = 1e9f;
  for (int s = -1; s <= 1; s += 2) for (int t = -1; t <= 1; t++)
    c = std::min(c, sdCapsule(p, V3(0.19f, 0.045f, s * 0.178f + t * 0.040f), V3(0.305f, 0.030f, s * 0.178f + t * 0.062f), 0.036f, 0.022f));
  for (int s = -1; s <= 1; s += 2) for (int t = 0; t < 2; t++)
    c = std::min(c, sdCapsule(p, V3(0.285f, 0.555f, s * 0.150f), V3(0.325f, 0.535f + t * 0.04f, s * 0.145f), 0.020f, 0.012f));
  // row of plates down the back
  float sp = 1e9f;
  const float spx[11][3] = {{0.10f, 1.005f, 0.034f}, {0.03f, 0.955f, 0.042f}, {-0.02f, 0.86f, 0.050f}, {-0.075f, 0.755f, 0.056f}, {-0.14f, 0.655f, 0.058f}, {-0.215f, 0.555f, 0.056f},
                            {-0.30f, 0.455f, 0.050f}, {-0.385f, 0.365f, 0.044f}, {-0.47f, 0.275f, 0.037f}, {-0.55f, 0.20f, 0.030f}, {-0.63f, 0.135f, 0.024f}};
  for (int k = 0; k < 11; k++) { V3 q = p - V3(spx[k][0], spx[k][1], spx[k][0] < -0.35f ? 0.03f + (-(spx[k][0] + 0.35f)) * 0.2f : 0.0f); float r = spx[k][2];
    sp = std::min(sp, sdEllipsoid(q, V3(0, 0, 0), V3(r * 1.0f, r * 1.15f, r * 0.42f))); }
  o.body = d; o.spikes = sp; o.claws = c;
  return o;
}
static void buildDino(Scene& sc) {
  Mesh& m = sc.dino;
  auto f = [](V3 p) { DinoParts q = dinoSDF(p); float d = smin(smin(q.body, q.spikes, 0.012f), q.claws, 0.018f); return smax(d, -p.y, 0.012f); };
  Mesh b; b.set(M_SKIN, RGBA(0.3f, 0.6f, 0.2f));
  sdfMesh(b, T(0, 0, 0), f, V3(-0.84f, -0.01f, -0.36f), V3(0.50f, 1.12f, 0.36f), 0.0040f);
  // paint: per-triangle colour from where the triangle sits and which way it faces
  for (size_t i = 0; i < b.ntri(); i++) {
    V3 p = (b.p[b.idx[i * 3]] + b.p[b.idx[i * 3 + 1]] + b.p[b.idx[i * 3 + 2]]) / 3.0f; V3 n = norm(b.n[b.idx[i * 3]] + b.n[b.idx[i * 3 + 1]] + b.n[b.idx[i * 3 + 2]]);
    DinoParts q = dinoSDF(p);
    V3 green(0.26f, 0.56f, 0.20f), dark(0.10f, 0.33f, 0.15f), belly(0.93f, 0.85f, 0.52f), orange(0.95f, 0.50f, 0.12f), claw(0.96f, 0.93f, 0.82f);
    float nz = fbm(p * 9.0f, 3);
    float stripe = smooth(0.15f, 0.45f, sinf(p.x * 21.0f + p.y * 6.0f + nz * 3.0f)) * smooth(0.05f, 0.5f, n.y + 0.3f * fabsf(n.z) + 0.25f);
    V3 c = lerp(green, dark, stripe * 0.85f);
    float under = smooth(0.25f, 0.75f, -n.y * 0.9f + n.x * 0.75f - fabsf(n.z) * 0.55f + (p.y < 0.72f ? 0.12f : -0.25f));
    if (p.x < -0.12f) under = smooth(0.45f, 0.85f, -n.y);
    c = lerp(c, belly, under);
    if (q.spikes < q.body + 0.004f && q.spikes < 0.02f) c = orange;
    if (q.claws < q.body - 0.002f && q.claws < 0.012f && (p.x > 0.235f || p.y > 0.3f)) c = claw;
    // inside of the mouth line
    if (p.x > 0.12f && fabsf(p.y - (0.826f - (p.x - 0.12f) * 0.10f)) < 0.012f && fabsf(n.y) > 0.25f && p.x < 0.42f) c = V3(0.45f, 0.10f, 0.10f);
    b.tcol[i] = RGBA(c.x, c.y, c.z, 0.5f);
  }
  // eyes: white ball with a glossy black pupil; teeth along the upper jaw
  for (int s = -1; s <= 1; s += 2) {
    b.set(M_PLASTIC, RGBA(0.95f, 0.95f, 0.92f, 0.1f)); ellipsoid(b, T(0.215f, 0.935f, s * 0.105f), V3(0.046f, 0.046f, 0.036f), 24, 12);
    b.set(M_PLASTIC, RGBA(0.02f, 0.02f, 0.02f, 0.0f)); ellipsoid(b, T(0.238f, 0.932f, s * 0.127f), V3(0.024f, 0.027f, 0.018f), 20, 10);
    b.set(M_PLASTIC, RGBA(0.97f, 0.96f, 0.90f, 0.2f));
    for (int k = 0; k < 6; k++) { float x = 0.16f + k * 0.039f; float zz = s * (0.068f - k * 0.004f);
      Mesh t; t.set(b.mat, b.col); lathe(t, T(0, 0, 0), Profile{{0, -0.030f}, {0.0085f, -0.004f}, {0.010f, 0.008f}, {0, 0.012f}}, 8); add(b, t, T(x, 0.802f - k * 0.004f, zz)); }
  }
  add(m, b, S(DINO_S));
  for (auto& t : m.t) t = t * DINO_S;
  // where does it come to rest when it topples onto its -z flank (about the outer edge of that foot)? Work in mirrored z.
  float pz = 0; for (auto& p : m.p) if (p.y < 0.004f && p.x > -0.03f) pz = std::max(pz, -p.z);
  sc.dinoAnim.pivotZ = pz - 0.002f;
  float rest = rad(88);
  for (float a = rad(40); a < rad(125); a += rad(0.5f)) {
    float mn = 1e9f; float ca = cosf(a), sa = sinf(a);
    for (auto& p : m.p) { float z = -p.z - sc.dinoAnim.pivotZ; if (z > -0.004f && p.y < 0.012f) continue; mn = std::min(mn, ca * p.y - sa * z); }
    if (mn < -0.0015f) { rest = a - rad(0.5f); break; }
  }
  sc.dinoAnim.fallAngle = rest;
  // where does the robot's round bumper first touch it? (robot drives +z along x = DINO_POS.x)
  float phi = rad(90) + DINO_YAW0, cz = 1e9f;
  for (auto& p : m.p) { if (p.y > ROBOT_TOP) continue; float wx = p.x * cosf(phi) + p.z * sinf(phi), wz = -p.x * sinf(phi) + p.z * cosf(phi);
    if (fabsf(wx) < ROBOT_R) cz = std::min(cz, wz - sqrtf(ROBOT_R * ROBOT_R - wx * wx)); }
  float toe = -cz - ROBOT_R;
  if (fabsf(toe - DINO_TOE) > 0.0015f) fprintf(stderr, "note: bumper contact measured at %.4f m, path.h DINO_TOE is %.4f\n", toe, DINO_TOE);
}

static Scene buildScene(bool lowDetail = false) {
  Scene sc; Mesh& m = sc.world;
  buildRoom(m, sc); buildFloor(m); if (!lowDetail) buildRug(m);
  buildSofa(m); buildCoffeeTable(m); buildDining(m); buildBookshelf(m); buildSideboard(m); buildLampAndArt(m);
  buildCurtains(m, sc); buildPlant(m); buildToys(m, sc); buildOutside(m);
  buildRobot(sc); buildDino(sc);
  return sc;
}
