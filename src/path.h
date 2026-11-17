// path.h — the robot's run: drive/turn commands integrated with acceleration limits, suspension over the rug edge,
// motor vibration, the camera rig on top of it, the toy dinosaur's reaction, and the event timeline for the sound track.
#pragma once
#include "base.h"

// ---- layout constants shared with the scene (metres; x east, y up, z south)
static const float ROOM_X = 6.2f, ROOM_Z = 4.4f, ROOM_H = 2.7f, WALL_T = 0.25f;
static const float RUG_X0 = 0.80f, RUG_X1 = 3.20f, RUG_Z0 = 1.12f, RUG_Z1 = 2.56f, RUG_H = 0.011f;
static const float SOFA_X0 = 0.90f, SOFA_X1 = 3.10f, SOFA_Z0 = 0.14f, SOFA_Z1 = 1.04f, SOFA_CLEAR = 0.175f;
static const float TABLE_X = 4.60f, TABLE_Z = 2.25f;
static const float PLANK_W = 0.14f;
static const float ROBOT_R = 0.17f, ROBOT_TOP = 0.090f;
static const float CAM_FWD = 0.108f, CAM_UP = 0.118f, CAM_PITCH = -5.0f; // camera on the deck: 6 cm behind the front edge
static const V3 DINO_POS(4.60f, 0, 3.80f);
static const float DINO_YAW0 = 62.0f * 3.14159265f / 180.0f; // it stands three-quarters on to the robot, looking towards the west wall
static const float DINO_TOE = 0.0549f; // the robot's bumper meets the toy when its edge is this far short of DINO_POS (buildDino prints the measured value)
static const float FPS = 30.0f;

struct RState { float x, z, th, v, om, pitch, roll, dy, vib; };

struct Ev { float t; std::string type; float a; };

struct Sim {
  static constexpr float DT = 1.0f / 240.0f;
  std::vector<RState> s; std::vector<Ev> ev;
  float tBump = 0, tEnd = 0, tReverse = 0;
  float x = 0.30f, z = 2.95f, th = 0, v = 0, om = 0, t = 0;
  float pitch = 0, pitchV = 0, hm = 0, hmV = 0, kick = 0, kickV = 0;
  Rng rng{77};
  float prevSeamF = 0, prevSeamM = 0; bool onRugF = false, onRugM = false;

  static float rugH(float px, float pz) { // height of the rug surface under a wheel (soft 2 cm ramp at the edge)
    float d = std::min(std::min(px - RUG_X0, RUG_X1 - px), std::min(pz - RUG_Z0, RUG_Z1 - pz));
    return RUG_H * smooth(-0.004f, 0.016f, d);
  }
  void step() {
    float fx = cosf(th), fz = -sinf(th);
    x += fx * v * DT; z += fz * v * DT; th += om * DT; t += DT;
    // wheels: front caster 12 cm ahead, drive wheels at the centre
    float cfx = x + fx * 0.12f, cfz = z + fz * 0.12f;
    float hF = rugH(cfx, cfz), hM = rugH(x, z);
    bool rf = hF > RUG_H * 0.5f, rm = hM > RUG_H * 0.5f;
    if (rf != onRugF) { ev.push_back({t, rf ? "rug_on" : "rug_off", 1.0f}); kickV += rf ? 0.30f : -0.18f; onRugF = rf; }
    if (rm != onRugM) { ev.push_back({t, rm ? "rug_on2" : "rug_off2", 0.7f}); kickV += rm ? -0.20f : 0.14f; onRugM = rm; }
    // plank seams (planks run along x): a tick when a wheel crosses z = k * PLANK_W on bare floor
    float sF = floorf(cfz / PLANK_W), sM = floorf(z / PLANK_W);
    if (sF != prevSeamF && !rf && fabsf(v) > 0.05f) { ev.push_back({t, "seam", 0.6f + 0.4f * rng.f()}); kickV += 0.035f * (rng.f() - 0.3f); }
    if (sM != prevSeamM && !rm && fabsf(v) > 0.05f) { ev.push_back({t, "seam", 0.9f + 0.4f * rng.f()}); kickV -= 0.025f * (rng.f() - 0.3f); }
    prevSeamF = sF; prevSeamM = sM;
    // suspension: body pitch follows the wheel heights through a spring; 'kick' is a fast ringing on top
    float target = atan2f(hF - hM, 0.12f);
    float wn = 34.0f, zeta = 0.42f;
    pitchV += (wn * wn * (target - pitch) - 2 * zeta * wn * pitchV) * DT; pitch += pitchV * DT;
    hmV += (wn * wn * (hM - hm) - 2 * 0.6f * wn * hmV) * DT; hm += hmV * DT;
    float wk = 75.0f; kickV += (-wk * wk * kick - 2 * 0.22f * wk * kickV) * DT; kick += kickV * DT;
    float mv = clampf(fabsf(v) / 0.3f + fabsf(om) / 1.3f, 0, 1);
    RState r; r.x = x; r.z = z; r.th = th; r.v = v; r.om = om;
    r.pitch = pitch + kick; r.roll = 0; r.dy = hm; r.vib = mv * (rm ? 1.6f : 1.0f);
    s.push_back(r);
  }
  void wait(float T) { int n = (int)(T / DT); for (int i = 0; i < n; i++) { v *= 0.9f; om *= 0.9f; step(); } v = 0; om = 0; }
  void drive(float dist, float vmax, bool hardStop = false) {
    float sgn = dist < 0 ? -1.0f : 1.0f; float D = fabsf(dist), done = 0, acc = 0.45f;
    while (done < D - 1e-4f) {
      float rem = D - done; float onRug = rugH(x, z) > RUG_H * 0.5f ? 0.80f : 1.0f;
      float vt = hardStop ? vmax * onRug : std::min(vmax * onRug, sqrtf(2 * acc * rem) + 0.01f);
      float sp = fabsf(v); sp += clampf(vt - sp, -acc * DT, acc * DT); v = sgn * sp;
      done += sp * DT; step();
      if (s.size() > 400000) break;
    }
    if (hardStop) { while (fabsf(v) > 0.004f) { float sp = fabsf(v) - 2.6f * DT; v = sgn * std::max(sp, 0.0f); step(); } }
    v = 0;
  }
  void turn(float deg, float wmax) {
    float sgn = deg < 0 ? -1.0f : 1.0f; float A = fabsf(rad(deg)), done = 0, acc = rad(210); float w = rad(wmax);
    while (done < A - 1e-4f) {
      float rem = A - done; float wt = std::min(w, sqrtf(2 * acc * rem) + 0.02f);
      float sp = fabsf(om); sp += clampf(wt - sp, -acc * DT, acc * DT); om = sgn * sp;
      done += sp * DT; step();
    }
    om = 0; th = roundf(th / (PI / 2)) * (PI / 2);
  }
  Sim() {
    const float V = 0.32f;
    wait(2.7f); ev.push_back({0.35f, "boot", 1}); ev.push_back({2.55f, "motor_on", 1});
    drive(1.75f, V); wait(0.12f);
    turn(90, 78); wait(0.12f);
    drive(2.95f - 0.60f, V); wait(0.15f);        // north: onto the rug, under the coffee table, under the sofa
    turn(-90, 78); wait(0.12f);
    drive(TABLE_X - 2.05f, V); wait(0.12f);      // east: out from under the sofa, along the bookshelf
    turn(-90, 78); wait(0.12f);
    float contactZ = DINO_POS.z - DINO_TOE - ROBOT_R;
    drive(contactZ - 0.60f, V, true);            // south: under the dining table, into the dinosaur
    tBump = t - 0.10f; ev.push_back({tBump, "bump", 1});
    wait(2.05f);
    tReverse = t; ev.push_back({t, "reverse", 1});
    drive(-0.24f, 0.20f); wait(0.2f);
    turn(-90, 70); wait(0.12f);
    drive(1.9f, V);
    tEnd = t;
  }
  RState at(float tt) const {
    float f = clampf(tt, 0, tEnd - 2 * DT) / DT; int i = (int)f; float u = f - i;
    const RState& a = s[std::min((size_t)i, s.size() - 1)]; const RState& b = s[std::min((size_t)i + 1, s.size() - 1)];
    RState r; r.x = lerpf(a.x, b.x, u); r.z = lerpf(a.z, b.z, u); r.th = lerpf(a.th, b.th, u); r.v = lerpf(a.v, b.v, u); r.om = lerpf(a.om, b.om, u);
    r.pitch = lerpf(a.pitch, b.pitch, u); r.roll = lerpf(a.roll, b.roll, u); r.dy = lerpf(a.dy, b.dy, u); r.vib = lerpf(a.vib, b.vib, u);
    return r;
  }
};

// smooth band-limited noise for vibration
static inline float snoise1(float t, int ch) { int i = (int)floorf(t); float f = t - i; f = f * f * (3 - 2 * f);
  return lerpf(hash3i(i, ch, 5), hash3i(i + 1, ch, 5), f) * 2 - 1; }

struct Rig { V3 base, fwd, up, right; V3 eye, cF, cU, cR; };
// robot frame and camera frame at time t
static inline Rig rigAt(const Sim& sim, float t) {
  RState r = sim.at(t);
  // motor/wheel vibration: tiny, fast
  float vp = (snoise1(t * 47.0f, 1) * 0.6f + snoise1(t * 23.0f, 2) * 0.4f) * rad(0.035f) * r.vib;
  float vr = (snoise1(t * 41.0f, 3) * 0.6f + snoise1(t * 19.0f, 4) * 0.4f) * rad(0.045f) * r.vib;
  float vy = snoise1(t * 9.0f, 5) * rad(0.05f) * r.vib;
  float th = r.th + vy, ph = r.pitch + vp, ro = r.roll + vr;
  V3 f0(cosf(th), 0, -sinf(th)), r0(sinf(th), 0, cosf(th)), u0(0, 1, 0);
  V3 f1 = f0 * cosf(ph) + u0 * sinf(ph), u1 = u0 * cosf(ph) - f0 * sinf(ph);
  V3 r2 = r0 * cosf(ro) + u1 * sinf(ro), u2 = u1 * cosf(ro) - r0 * sinf(ro);
  Rig g; g.base = V3(r.x, r.dy, r.z); g.fwd = f1; g.up = u2; g.right = r2;
  g.eye = g.base + g.fwd * CAM_FWD + g.up * CAM_UP;
  float cp = rad(CAM_PITCH);
  g.cF = norm(g.fwd * cosf(cp) + g.up * sinf(cp)); g.cU = norm(g.up * cosf(cp) - g.fwd * sinf(cp)); g.cR = g.right;
  return g;
}
// robot-local (x forward, y up, z right) -> world
static inline M34 robotXf(const Rig& g) {
  M34 m; m.r[0] = g.fwd.x; m.r[1] = g.up.x; m.r[2] = g.right.x; m.r[3] = g.fwd.y; m.r[4] = g.up.y; m.r[5] = g.right.y; m.r[6] = g.fwd.z; m.r[7] = g.up.z; m.r[8] = g.right.z;
  m.t = g.base; return m;
}
// robot-local -> camera space (x right, y up, z forward); fixed, because the camera is bolted to the deck
static inline V3 robotToCam(V3 p) {
  float cp = rad(CAM_PITCH); V3 d(p.x - CAM_FWD, p.y - CAM_UP, p.z);
  V3 cF(cosf(cp), sinf(cp), 0), cU(-sinf(cp), cosf(cp), 0), cR(0, 0, 1);
  return V3(dot(d, cR), dot(d, cU), dot(d, cF));
}
static inline V3 robotDirToCam(V3 n) {
  float cp = rad(CAM_PITCH); V3 cF(cosf(cp), sinf(cp), 0), cU(-sinf(cp), cosf(cp), 0), cR(0, 0, 1);
  return V3(dot(n, cR), dot(n, cU), dot(n, cF));
}

// the toy dinosaur: stands facing the robot, gets shoved, rocks, then topples onto its side. Local frame: +x forward, +y up.
struct DinoAnim { float fallAngle = rad(86); float pivotZ = 0.026f; };
static inline M34 dinoXf(const Sim& sim, const DinoAnim& da, float t, float* fallT = nullptr) {
  float tb = t - sim.tBump;
  float slide = 0, yaw = 0, rock = 0, tip = 0;
  if (tb > 0) {
    slide = 0.032f * (1 - expf(-tb * 9.0f));
    yaw = rad(13) * (1 - expf(-tb * 7.0f));
    rock = rad(7.5f) * expf(-tb * 3.2f) * sinf(tb * 2 * PI * 3.1f);
    float tf = tb - 0.78f; // starts to go over
    float T = 0.50f;       // time to fall
    if (fallT) *fallT = sim.tBump + 0.78f + T;
    if (tf > 0) {
      if (tf < T) { float u = tf / T; tip = da.fallAngle * (0.18f * u + 0.82f * u * u * u); }
      else { float tr = tf - T; tip = da.fallAngle - rad(5.5f) * expf(-tr * 9.0f) * fabsf(sinf(tr * 2 * PI * 4.2f)); }
    }
  }
  M34 place = T(DINO_POS + V3(0.012f * slide / 0.032f, 0, slide)) * RY(rad(90) + DINO_YAW0 + yaw);
  M34 rockM = RZ(rock * (tip > 0 ? expf(-tip * 6) : 1.0f)) * RX(-0.6f * rock * (tip > 0 ? expf(-tip * 6) : 1.0f)); // nods and sways about the feet
  // goes over onto the flank that faces away from the robot (local -z), pivoting on the outer edge of that foot
  M34 tipM = T(0, 0, -da.pivotZ) * RX(-tip) * T(0, 0, da.pivotZ);
  return place * tipM * rockM;
}

// focus distance the camera wants at time t (it hunts slowly, like a small autofocus)
static inline float focusTarget(const Sim& sim, float t) {
  RState r = sim.at(t); Rig g = rigAt(sim, t);
  float d = len(V3(DINO_POS.x, 0.09f, DINO_POS.z) - g.eye);
  float facing = dot(norm(V3(DINO_POS.x, 0.09f, DINO_POS.z) - g.eye), g.cF);
  float f = 1.6f;
  if (facing > 0.75f && d < 1.3f) f = lerpf(clampf(d - 0.02f, 0.085f, 1.6f), 1.6f, smooth(0.9f, 1.3f, d));
  bool underSofa = r.x > SOFA_X0 && r.x < SOFA_X1 && r.z < SOFA_Z1 + 0.1f;
  if (underSofa) f = std::min(f, 0.75f);
  return f;
}
