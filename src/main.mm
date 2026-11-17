#include <map>
// main.mm — host: builds the apartment, hands it to the Metal path tracer, bakes the bounce-light grid,
// renders frames, applies the camera post-processing on the CPU and pipes frames to ffmpeg.
#import <Foundation/Foundation.h>
#import <Metal/Metal.h>
#include <dispatch/dispatch.h>
#include <sys/stat.h>
#include "scene.h"

struct Uni {
  float camP0[4], camP1[4], camR0[4], camU0[4], camF0[4], camR1[4], camU1[4], camF1[4];
  float lens[4], sunDir[4], sunCol[4], roomLo[4], roomHi[4], gridLo[4], gridInv[4], portal[4][4], misc[4], misc2[4];
  u32 counts[4], dims[4], grid[4];
};
struct InstInfo { float r0[4], r1[4], r2[4]; u32 meta[4]; };
static void set3(float* d, V3 v, float w = 0) { d[0] = v.x; d[1] = v.y; d[2] = v.z; d[3] = w; }
static double nowSec() { return [[NSDate date] timeIntervalSince1970]; }
static uint16_t f2h(float f) { // float -> half
  uint32_t x; memcpy(&x, &f, 4); uint32_t sign = (x >> 16) & 0x8000; int e = (int)((x >> 23) & 0xff) - 127 + 15; uint32_t m = x & 0x7fffff;
  if (e <= 0) return (uint16_t)sign; if (e >= 31) return (uint16_t)(sign | 0x7bff);
  return (uint16_t)(sign | (e << 10) | (m >> 13));
}

// ---------------------------------------------------------------- CPU post-processing
static void blurSep(std::vector<float>& img, int W, int H, float sigma) {
  int R = (int)ceilf(sigma * 2.6f); std::vector<float> k(R + 1); float s = 0;
  for (int i = 0; i <= R; i++) { k[i] = expf(-0.5f * i * i / (sigma * sigma)); s += (i ? 2 : 1) * k[i]; }
  for (auto& x : k) x /= s;
  std::vector<float> tmpv(img.size()); float* tmp = tmpv.data(); float* ip = img.data(); const float* kk = k.data();
  dispatch_apply(H, dispatch_get_global_queue(QOS_CLASS_USER_INITIATED, 0), ^(size_t y) {
    for (int x = 0; x < W; x++) { float a[3] = {0, 0, 0};
      for (int i = -R; i <= R; i++) { int xx = std::min(std::max(x + i, 0), W - 1); float w = kk[abs(i)]; const float* p = &ip[(y * W + xx) * 3]; a[0] += p[0] * w; a[1] += p[1] * w; a[2] += p[2] * w; }
      float* o = &tmp[(y * W + x) * 3]; o[0] = a[0]; o[1] = a[1]; o[2] = a[2]; }
  });
  dispatch_apply(W, dispatch_get_global_queue(QOS_CLASS_USER_INITIATED, 0), ^(size_t x) {
    for (int y = 0; y < H; y++) { float a[3] = {0, 0, 0};
      for (int i = -R; i <= R; i++) { int yy = std::min(std::max(y + i, 0), H - 1); float w = kk[abs(i)]; const float* p = &tmp[(yy * W + x) * 3]; a[0] += p[0] * w; a[1] += p[1] * w; a[2] += p[2] * w; }
      float* o = &ip[(y * W + x) * 3]; o[0] = a[0]; o[1] = a[1]; o[2] = a[2]; }
  });
}
static inline float toneCh(float x) { // filmic curve, then sRGB encoding
  x = fmaxf(x, 0.0f) * 0.78f;
  float y = (x * (2.51f * x + 0.03f)) / (x * (2.43f * x + 0.59f) + 0.14f); y = clampf(y, 0, 1);
  return y <= 0.0031308f ? 12.92f * y : 1.055f * powf(y, 1 / 2.4f) - 0.055f;
}
struct PostP { float exposure = 1, fade = 1, grain = 1; u32 seed = 0; bool raw = false; };
// linear image = sharp light + averaged surface colour x soft light (E is float4 per pixel, already normalised)
static void combine(const float* acc, const float* kd, const float* E, int W, int H, std::vector<float>& imgv) {
  imgv.resize((size_t)W * H * 3); float* img = imgv.data();
  dispatch_apply(H, dispatch_get_global_queue(QOS_CLASS_USER_INITIATED, 0), ^(size_t y) {
    for (size_t i = y * W; i < (y + 1) * W; i++) { float inv = 1.0f / fmaxf(acc[i * 4 + 3], 1.0f);
      for (int c = 0; c < 3; c++) img[i * 3 + c] = (acc[i * 4 + c] + kd[i * 4 + c] * E[i * 4 + c]) * inv; }
  });
}
static void postProcess(std::vector<float>& imgv, int W, int H, const PostP& P, std::vector<uint8_t>& out) {
  float* img = imgv.data();
  for (size_t i = 0; i < (size_t)W * H * 3; i++) img[i] *= P.exposure;
  out.resize((size_t)W * H * 3); uint8_t* op = out.data();
  if (P.raw) { for (size_t i = 0; i < (size_t)W * H * 3; i++) op[i] = (uint8_t)(255 * clampf(powf(fmaxf(img[i], 0), 1 / 2.2f), 0, 1)); return; }
  // glare: three blur scales at quarter resolution
  int ds = std::max(1, W / 480); int w4 = W / ds, h4 = H / ds;
  std::vector<float> b1((size_t)w4 * h4 * 3);
  for (int y = 0; y < h4; y++) for (int x = 0; x < w4; x++) { float a[3] = {0, 0, 0};
    for (int j = 0; j < ds; j++) for (int i = 0; i < ds; i++) { const float* p = &img[((size_t)(y * ds + j) * W + x * ds + i) * 3]; a[0] += p[0]; a[1] += p[1]; a[2] += p[2]; }
    for (int c = 0; c < 3; c++) b1[((size_t)y * w4 + x) * 3 + c] = fminf(a[c] / (ds * ds), 30.0f); }
  std::vector<float> b2 = b1, b3 = b1;
  blurSep(b1, w4, h4, 1.6f); blurSep(b2, w4, h4, 7.0f); blurSep(b3, w4, h4, 26.0f);
  const float* B1 = b1.data(); const float* B2 = b2.data(); const float* B3 = b3.data(); const PostP PP = P;
  dispatch_apply(H, dispatch_get_global_queue(QOS_CLASS_USER_INITIATED, 0), ^(size_t y) {
    u32 rs = pcg((u32)y * 7919u + PP.seed * 104729u + 11u);
    for (int x = 0; x < W; x++) {
      float nx = (x + 0.5f) / W * 2 - 1, ny = ((y + 0.5f) / H * 2 - 1) * (float)H / W; float r2 = (nx * nx + ny * ny) / (1 + (float)H * H / ((float)W * W));
      // lateral chromatic aberration: red and blue sampled at slightly different magnification
      float c[3];
      for (int ch = 0; ch < 3; ch++) {
        float sc = ch == 0 ? 1.0f + 0.0016f * r2 : (ch == 2 ? 1.0f - 0.0016f * r2 : 1.0f);
        float sx = ((nx * sc) * 0.5f + 0.5f) * W - 0.5f, sy = ((ny * sc * (float)W / H) * 0.5f + 0.5f) * H - 0.5f;
        int x0 = (int)floorf(sx), y0 = (int)floorf(sy); float fx = sx - x0, fy = sy - y0;
        int xa = std::min(std::max(x0, 0), W - 1), xb = std::min(std::max(x0 + 1, 0), W - 1), ya = std::min(std::max(y0, 0), H - 1), yb = std::min(std::max(y0 + 1, 0), H - 1);
        c[ch] = lerpf(lerpf(img[((size_t)ya * W + xa) * 3 + ch], img[((size_t)ya * W + xb) * 3 + ch], fx), lerpf(img[((size_t)yb * W + xa) * 3 + ch], img[((size_t)yb * W + xb) * 3 + ch], fx), fy);
      }
      // glare (bilinear from the quarter-res blurs)
      float gx = (x + 0.5f) / ds - 0.5f, gy = (y + 0.5f) / ds - 0.5f; int g0 = std::min(std::max((int)floorf(gx), 0), w4 - 1), g1 = std::min(g0 + 1, w4 - 1), h0 = std::min(std::max((int)floorf(gy), 0), h4 - 1), h1 = std::min(h0 + 1, h4 - 1);
      float fx = clampf(gx - g0, 0, 1), fy = clampf(gy - h0, 0, 1);
      for (int ch = 0; ch < 3; ch++) {
        auto S = [&](const float* B) { return lerpf(lerpf(B[((size_t)h0 * w4 + g0) * 3 + ch], B[((size_t)h0 * w4 + g1) * 3 + ch], fx), lerpf(B[((size_t)h1 * w4 + g0) * 3 + ch], B[((size_t)h1 * w4 + g1) * 3 + ch], fx), fy); };
        c[ch] = c[ch] * 0.885f + S(B1) * 0.050f + S(B2) * 0.040f + S(B3) * 0.025f;
      }
      float vig = 1.0f - 0.30f * r2 - 0.10f * r2 * r2;
      float l = 0.2126f * c[0] + 0.7152f * c[1] + 0.0722f * c[2];
      float desat = 0.55f * smooth(1.5f, 12.0f, l); // very bright things lose colour, like a clipped sensor
      // grain: two correlated-ish gaussians, stronger in the shadows
      rs = pcg(rs); float n1 = ((rs & 0xffff) + ((rs >> 16) & 0xffff)) / 65535.0f - 1.0f; rs = pcg(rs); float n2 = ((rs & 0xffff) + ((rs >> 16) & 0xffff)) / 65535.0f - 1.0f;
      for (int ch = 0; ch < 3; ch++) {
        float v = lerpf(c[ch], l, desat) * vig * PP.fade;
        float t = toneCh(v);
        float gs = PP.grain * (0.020f - 0.012f * t);
        t += gs * (n1 * 0.85f + (ch == 0 ? n2 : (ch == 2 ? -n2 : 0.0f)) * 0.35f);
        rs = pcg(rs); float dth = ((rs >> 8) & 0xffff) / 65535.0f - 0.5f;
        op[((size_t)y * W + x) * 3 + ch] = (uint8_t)clampf(t * 255.0f + 0.5f + dth * 0.9f, 0, 255);
      }
    }
  });
}

// ---------------------------------------------------------------- Metal helpers
static id<MTLAccelerationStructure> buildAS(id<MTLDevice> dev, id<MTLCommandQueue> q, MTLAccelerationStructureDescriptor* d) {
  MTLAccelerationStructureSizes sz = [dev accelerationStructureSizesWithDescriptor:d];
  id<MTLAccelerationStructure> as = [dev newAccelerationStructureWithSize:sz.accelerationStructureSize];
  id<MTLBuffer> scratch = [dev newBufferWithLength:sz.buildScratchBufferSize options:MTLResourceStorageModePrivate];
  id<MTLCommandBuffer> cb = [q commandBuffer]; id<MTLAccelerationStructureCommandEncoder> e = [cb accelerationStructureCommandEncoder];
  [e buildAccelerationStructure:as descriptor:d scratchBuffer:scratch scratchBufferOffset:0]; [e endEncoding]; [cb commit]; [cb waitUntilCompleted];
  return as;
}
static void setInst(MTLAccelerationStructureInstanceDescriptor* d, InstInfo* info, int i, const M34& m, u32 asIndex, u32 mask, u32 firstTri) {
  memset(&d[i], 0, sizeof(d[i]));
  for (int c = 0; c < 3; c++) d[i].transformationMatrix.columns[c] = MTLPackedFloat3Make(m.r[c], m.r[3 + c], m.r[6 + c]);
  d[i].transformationMatrix.columns[3] = MTLPackedFloat3Make(m.t.x, m.t.y, m.t.z);
  d[i].mask = mask; d[i].accelerationStructureIndex = asIndex; d[i].options = MTLAccelerationStructureInstanceOptionOpaque;
  float* r[3] = {info[i].r0, info[i].r1, info[i].r2}; float tt[3] = {m.t.x, m.t.y, m.t.z};
  for (int k = 0; k < 3; k++) { r[k][0] = m.r[k * 3]; r[k][1] = m.r[k * 3 + 1]; r[k][2] = m.r[k * 3 + 2]; r[k][3] = tt[k]; }
  info[i].meta[0] = firstTri; info[i].meta[1] = info[i].meta[2] = info[i].meta[3] = 0;
}

int main(int argc, char** argv) {
  @autoreleasepool {
    int W = 1920, H = 1080, spp = 16, from = 0, to = -1, step = 1, mode = 0, sppPass = 4, dnIter = 5;
    const char *segOut = nullptr, *framesDir = nullptr, *meterOut = nullptr, *timelineOut = nullptr, *expoFile = nullptr, *giCache = "work/gi.bin";
    bool collide = false, freeCam = false, noGI = false, lowDetail = false, raw = false, rebake = false; V3 eye, at; float hfov = 112, fixedExpo = -1, fogD = 0.010f, tFree = 0, aper = 0.0007f, focusOv = -1, clampI = 2.5f;
    for (int i = 1; i < argc; i++) {
      std::string a = argv[i]; auto F = [&](int k) { return (float)atof(argv[i + k]); };
      if (a == "--w") W = atoi(argv[++i]); else if (a == "--h") H = atoi(argv[++i]); else if (a == "--spp") spp = atoi(argv[++i]);
      else if (a == "--from") from = atoi(argv[++i]); else if (a == "--to") to = atoi(argv[++i]); else if (a == "--step") step = atoi(argv[++i]);
      else if (a == "--seg") segOut = argv[++i]; else if (a == "--frames") framesDir = argv[++i]; else if (a == "--meter") meterOut = argv[++i];
      else if (a == "--timeline") timelineOut = argv[++i]; else if (a == "--expo") expoFile = argv[++i]; else if (a == "--exposure") fixedExpo = (float)atof(argv[++i]);
      else if (a == "--eye") { eye = V3(F(1), F(2), F(3)); i += 3; freeCam = true; } else if (a == "--at") { at = V3(F(1), F(2), F(3)); i += 3; }
      else if (a == "--hfov") hfov = (float)atof(argv[++i]); else if (a == "--mode") mode = atoi(argv[++i]); else if (a == "--nogi") noGI = true;
      else if (a == "--low") lowDetail = true; else if (a == "--raw") raw = true; else if (a == "--fog") fogD = (float)atof(argv[++i]);
      else if (a == "--time") tFree = (float)atof(argv[++i]); else if (a == "--aperture") aper = (float)atof(argv[++i]); else if (a == "--focus") focusOv = (float)atof(argv[++i]);
      else if (a == "--rebake") rebake = true; else if (a == "--gi") giCache = argv[++i]; else if (a == "--clamp") clampI = (float)atof(argv[++i]);
      else if (a == "--collide") collide = true;
      else if (a == "--spppass") sppPass = atoi(argv[++i]); else if (a == "--dn") dnIter = std::min(std::max(atoi(argv[++i]), 0), 7);
      else { fprintf(stderr, "unknown argument %s\n", a.c_str()); return 2; }
    }
    double tStart = nowSec();
    Sim sim; int NF = (int)floorf(sim.tEnd * FPS);
    if (to < 0) to = NF - 1;
    fprintf(stderr, "run: %.2f s, %d frames; bump at %.2f s, reverse at %.2f s\n", sim.tEnd, NF, sim.tBump, sim.tReverse);

    Scene sc = buildScene(lowDetail);
    fprintf(stderr, "scene: world %zu tris, dino %zu, robot %zu, shell %zu; dino rests at %.1f deg (pivot z %.4f)  [%.1fs]\n", sc.world.ntri(), sc.dino.ntri(), sc.robot.ntri(), sc.shell.ntri(), sc.dinoAnim.fallAngle * 180 / PI, sc.dinoAnim.pivotZ, nowSec() - tStart);

    // ---- event timeline for the sound track
    if (timelineOut) {
      std::vector<Ev> ev = sim.ev; float fallT = 0; dinoXf(sim, sc.dinoAnim, sim.tBump + 5.0f, &fallT);
      ev.push_back({fallT, "dino_fall", 1}); ev.push_back({sim.tBump + 0.78f, "dino_tip", 1});
      for (V3 c : sc.cereal) { if (fabsf(c.x - TABLE_X) > 0.15f) continue;
        for (size_t i = 0; i < sim.s.size(); i++) { const RState& r = sim.s[i]; if (fabsf(r.x - TABLE_X) < 0.02f && r.v > 0.05f && r.z + 0.125f >= c.z && r.z < c.z) { ev.push_back({i * Sim::DT, "cereal", 1}); break; } } }
      std::sort(ev.begin(), ev.end(), [](const Ev& a, const Ev& b) { return a.t < b.t; });
      FILE* f = fopen(timelineOut, "w"); fprintf(f, "{\"fps\": %g, \"frames\": %d, \"duration\": %.4f, \"bump\": %.4f, \"reverse\": %.4f,\n\"events\": [", FPS, NF, NF / FPS, sim.tBump, sim.tReverse);
      for (size_t i = 0; i < ev.size(); i++) fprintf(f, "%s{\"t\": %.4f, \"type\": \"%s\", \"a\": %.3f}", i ? ", " : "", ev[i].t, ev[i].type.c_str(), ev[i].a);
      fprintf(f, "],\n\"state\": [");
      int n = (int)(sim.tEnd * 120);
      for (int i = 0; i < n; i++) { float t = i / 120.0f; RState r = sim.at(t);
        bool rug = Sim::rugH(r.x, r.z) > RUG_H * 0.5f; Rig g = rigAt(sim, t);
        float sofa = smooth(SOFA_Z1 + 0.10f, SOFA_Z1 - 0.12f, g.eye.z) * smooth(SOFA_X1 + 0.10f, SOFA_X1 - 0.12f, g.eye.x) * smooth(SOFA_X0 - 0.1f, SOFA_X0 + 0.12f, g.eye.x);
        float table = smooth(1.35f, 1.6f, g.eye.z) * smooth(3.15f, 2.9f, g.eye.z) * smooth(0.6f, 0.4f, fabsf(g.eye.x - TABLE_X));
        fprintf(f, "%s[%.3f, %.3f, %.3f, %d, %.3f, %.3f, %.3f, %.3f]", i ? ", " : "", r.v, r.om, r.vib, rug ? 1 : 0, sofa, table, r.x, r.z); }
      fprintf(f, "]}\n"); fclose(f);
      fprintf(stderr, "timeline: %zu events -> %s\n", ev.size(), timelineOut);
      if (!segOut && !framesDir && !meterOut) return 0;
    }

    // ---- route check: does the robot's body pass through anything it should not? (triangle bounding boxes against the body disc)
    if (collide) {
      struct Hc { float t; V3 p; int mat; int n; }; std::map<long, Hc> hits;
      const Mesh& m = sc.world; std::vector<u32> cand;
      for (u32 i = 0; i < m.ntri(); i++) { int mt = m.tmat[i]; if (mt == M_FLOOR || mt == M_RUG) continue;
        float lo = 1e9f, hi = -1e9f; for (int k = 0; k < 3; k++) { float y = m.p[m.idx[i * 3 + k]].y; lo = std::min(lo, y); hi = std::max(hi, y); }
        if (hi < 0.004f || lo > CAM_UP + 0.02f) continue; cand.push_back(i); }
      for (size_t si = 0; si < sim.s.size(); si += 6) { const RState& r = sim.s[si];
        for (u32 i : cand) { V3 a = m.p[m.idx[i * 3]], b = m.p[m.idx[i * 3 + 1]], c = m.p[m.idx[i * 3 + 2]];
          float x0 = std::min(a.x, std::min(b.x, c.x)), x1 = std::max(a.x, std::max(b.x, c.x)), z0 = std::min(a.z, std::min(b.z, c.z)), z1 = std::max(a.z, std::max(b.z, c.z));
          float dx = r.x - clampf(r.x, x0, x1), dz = r.z - clampf(r.z, z0, z1); if (dx * dx + dz * dz > ROBOT_R * ROBOT_R) continue;
          float lo = std::min(a.y, std::min(b.y, c.y)); float cx = clampf(r.x, x0, x1), cz = clampf(r.z, z0, z1);
          long key = (long)floorf(cx / 0.06f) * 100000 + (long)floorf(cz / 0.06f) * 100 + m.tmat[i];
          auto it = hits.find(key); if (it == hits.end()) hits[key] = {si * Sim::DT, V3(cx, lo, cz), (int)m.tmat[i], 1}; else it->second.n++; } }
      std::vector<Hc> hv; for (auto& kv : hits) hv.push_back(kv.second); std::sort(hv.begin(), hv.end(), [](const Hc& a, const Hc& b) { return a.t < b.t; });
      for (auto& h : hv) printf("t=%6.2f  at (%.2f, %.3f, %.2f)  material %d  (%d)\n", h.t, h.p.x, h.p.y, h.p.z, h.mat, h.n);
      printf("%zu contact cells\n", hv.size()); return 0;
    }

    // ---- Metal
    id<MTLDevice> dev = MTLCreateSystemDefaultDevice(); id<MTLCommandQueue> queue = [dev newCommandQueue];
    NSError* err = nil;
    NSString* dir = [[[NSString stringWithUTF8String:argv[0]] stringByDeletingLastPathComponent] stringByAppendingPathComponent:@"src"];
    NSString* src = [NSString stringWithContentsOfFile:[dir stringByAppendingPathComponent:@"render.metal"] encoding:NSUTF8StringEncoding error:&err];
    if (!src) { fprintf(stderr, "cannot read render.metal: %s\n", err.localizedDescription.UTF8String); return 1; }
    MTLCompileOptions* opt = [MTLCompileOptions new];
    id<MTLLibrary> lib = [dev newLibraryWithSource:src options:opt error:&err];
    if (!lib) { fprintf(stderr, "shader compile error:\n%s\n", err.localizedDescription.UTF8String); return 1; }
    id<MTLComputePipelineState> psoRender = [dev newComputePipelineStateWithFunction:[lib newFunctionWithName:@"render"] error:&err];
    id<MTLComputePipelineState> psoBake = [dev newComputePipelineStateWithFunction:[lib newFunctionWithName:@"bake"] error:&err];
    id<MTLComputePipelineState> psoPrep = [dev newComputePipelineStateWithFunction:[lib newFunctionWithName:@"dnPrep"] error:&err];
    id<MTLComputePipelineState> psoAtrous = [dev newComputePipelineStateWithFunction:[lib newFunctionWithName:@"dnAtrous"] error:&err];
    if (!psoRender || !psoBake || !psoPrep || !psoAtrous) { fprintf(stderr, "pipeline error: %s\n", err.localizedDescription.UTF8String); return 1; }

    // ---- geometry buffers: world | dino | robot | shell
    const Mesh* parts[4] = {&sc.world, &sc.dino, &sc.robot, &sc.shell};
    size_t nv = 0, nt = 0, vBase[4], tBase[4];
    for (int k = 0; k < 4; k++) { vBase[k] = nv; tBase[k] = nt; nv += parts[k]->p.size(); nt += parts[k]->ntri(); }
    id<MTLBuffer> bPos = [dev newBufferWithLength:nv * 12 options:MTLResourceStorageModeShared], bNrm = [dev newBufferWithLength:nv * 12 options:MTLResourceStorageModeShared], bTex = [dev newBufferWithLength:nv * 12 options:MTLResourceStorageModeShared];
    id<MTLBuffer> bIdx = [dev newBufferWithLength:nt * 12 options:MTLResourceStorageModeShared], bTri = [dev newBufferWithLength:nt * 8 options:MTLResourceStorageModeShared];
    for (int k = 0; k < 4; k++) {
      const Mesh& m = *parts[k];
      memcpy((float*)bPos.contents + vBase[k] * 3, m.p.data(), m.p.size() * 12); memcpy((float*)bNrm.contents + vBase[k] * 3, m.n.data(), m.n.size() * 12); memcpy((float*)bTex.contents + vBase[k] * 3, m.t.data(), m.t.size() * 12);
      u32* ip = (u32*)bIdx.contents + tBase[k] * 3; for (size_t i = 0; i < m.idx.size(); i++) ip[i] = m.idx[i] + (u32)vBase[k];
      u32* tp = (u32*)bTri.contents + tBase[k] * 2; for (size_t i = 0; i < m.ntri(); i++) { tp[i * 2] = m.tmat[i]; tp[i * 2 + 1] = m.tcol[i]; }
    }
    id<MTLAccelerationStructure> blas[4];
    for (int k = 0; k < 4; k++) {
      MTLAccelerationStructureTriangleGeometryDescriptor* g = [MTLAccelerationStructureTriangleGeometryDescriptor descriptor];
      g.vertexBuffer = bPos; g.vertexStride = 12; g.vertexFormat = MTLAttributeFormatFloat3; g.indexBuffer = bIdx; g.indexBufferOffset = tBase[k] * 12; g.indexType = MTLIndexTypeUInt32; g.triangleCount = parts[k]->ntri();
      MTLPrimitiveAccelerationStructureDescriptor* pd = [MTLPrimitiveAccelerationStructureDescriptor descriptor]; pd.geometryDescriptors = @[g];
      blas[k] = buildAS(dev, queue, pd);
    }
    const int NI = 3;
    id<MTLBuffer> bInstDesc = [dev newBufferWithLength:sizeof(MTLAccelerationStructureInstanceDescriptor) * NI options:MTLResourceStorageModeShared];
    id<MTLBuffer> bInst = [dev newBufferWithLength:sizeof(InstInfo) * NI options:MTLResourceStorageModeShared];
    MTLInstanceAccelerationStructureDescriptor* idesc = [MTLInstanceAccelerationStructureDescriptor descriptor];
    idesc.instancedAccelerationStructures = @[blas[0], blas[1], blas[2]]; idesc.instanceCount = NI; idesc.instanceDescriptorBuffer = bInstDesc;
    MTLAccelerationStructureSizes tsz = [dev accelerationStructureSizesWithDescriptor:idesc];
    id<MTLAccelerationStructure> tlas = [dev newAccelerationStructureWithSize:tsz.accelerationStructureSize + 4096];
    id<MTLBuffer> tScratch = [dev newBufferWithLength:tsz.buildScratchBufferSize + 4096 options:MTLResourceStorageModePrivate];
    auto buildTLAS = [&](const M34& dinoM, const M34& robotM) {
      MTLAccelerationStructureInstanceDescriptor* d = (MTLAccelerationStructureInstanceDescriptor*)bInstDesc.contents; InstInfo* info = (InstInfo*)bInst.contents;
      setInst(d, info, 0, M34(), 0, 1, (u32)tBase[0]); setInst(d, info, 1, dinoM, 1, 1, (u32)tBase[1]); setInst(d, info, 2, robotM, 2, 2, (u32)tBase[2]);
      id<MTLCommandBuffer> cb = [queue commandBuffer]; id<MTLAccelerationStructureCommandEncoder> e = [cb accelerationStructureCommandEncoder];
      [e buildAccelerationStructure:tlas descriptor:idesc scratchBuffer:tScratch scratchBufferOffset:0]; [e endEncoding]; [cb commit]; [cb waitUntilCompleted];
    };
    fprintf(stderr, "buffers + acceleration structures ready: %zu vertices, %zu triangles  [%.1fs]\n", nv, nt, nowSec() - tStart);

    // ---- uniforms that do not change
    id<MTLBuffer> bUni = [dev newBufferWithLength:sizeof(Uni) options:MTLResourceStorageModeShared];
    Uni* U = (Uni*)bUni.contents; memset(U, 0, sizeof(Uni));
    const float PZ = ROOM_Z + WALL_T;
    set3(U->sunDir, sc.sun, cosf(rad(0.45f))); set3(U->sunCol, V3(7.5f, 6.5f, 5.3f));
    set3(U->roomLo, V3(-0.02f, -0.02f, -0.02f)); set3(U->roomHi, V3(ROOM_X + 0.02f, ROOM_H + 0.02f, PZ));
    float aTot = 0; for (int k = 0; k < sc.nPortal; k++) { for (int j = 0; j < 4; j++) U->portal[k][j] = sc.portal[k][j]; aTot += (sc.portal[k][1] - sc.portal[k][0]) * (sc.portal[k][3] - sc.portal[k][2]); }
    U->misc[1] = clampI; U->misc[2] = fogD; U->misc[3] = aTot * fabsf(sc.sun.z);
    U->misc2[0] = aTot; U->misc2[1] = 1.0f; U->misc2[2] = freeCam ? 0.0f : 1.0f;
    U->counts[1] = sc.nPortal; U->counts[3] = (u32)tBase[3];
    const float cell = 0.08f; int gx = (int)ceilf(ROOM_X / cell) + 2, gy = (int)ceilf(ROOM_H / cell) + 2, gz = (int)ceilf(PZ / cell) + 2;
    set3(U->gridLo, V3(-cell, -cell, -cell)); set3(U->gridInv, V3(1.0f / (gx * cell), 1.0f / (gy * cell), 1.0f / (gz * cell)));
    U->grid[0] = gx; U->grid[1] = gy; U->grid[2] = gz;

    // ---- bounce-light grid
    id<MTLTexture> gtex[6];
    { MTLTextureDescriptor* td = [MTLTextureDescriptor new]; td.textureType = MTLTextureType3D; td.pixelFormat = MTLPixelFormatRGBA16Float; td.width = gx; td.height = gy; td.depth = gz; td.usage = MTLTextureUsageShaderRead; td.storageMode = MTLStorageModeShared;
      for (int k = 0; k < 6; k++) gtex[k] = [dev newTextureWithDescriptor:td]; }
    size_t nvox = (size_t)gx * gy * gz; std::vector<float> gridE(nvox * 6 * 3, 0.0f);
    auto uploadGrid = [&]() {
      std::vector<uint16_t> hb(nvox * 4);
      for (int k = 0; k < 6; k++) { for (size_t v = 0; v < nvox; v++) { for (int c = 0; c < 3; c++) hb[v * 4 + c] = f2h(gridE[(v * 6 + k) * 3 + c]); hb[v * 4 + 3] = f2h(1.0f); }
        [gtex[k] replaceRegion:MTLRegionMake3D(0, 0, 0, gx, gy, gz) mipmapLevel:0 slice:0 withBytes:hb.data() bytesPerRow:gx * 8 bytesPerImage:gx * gy * 8]; }
    };
    uploadGrid();
    auto bindCommon = [&](id<MTLComputeCommandEncoder> ce) {
      [ce setBuffer:bUni offset:0 atIndex:1]; [ce setAccelerationStructure:tlas atBufferIndex:2];
      [ce setBuffer:bIdx offset:0 atIndex:3]; [ce setBuffer:bPos offset:0 atIndex:4]; [ce setBuffer:bNrm offset:0 atIndex:5]; [ce setBuffer:bTex offset:0 atIndex:6];
      [ce setBuffer:bTri offset:0 atIndex:7]; [ce setBuffer:bInst offset:0 atIndex:8]; [ce setAccelerationStructure:blas[3] atBufferIndex:9];
      for (int k = 0; k < 3; k++) [ce useResource:blas[k] usage:MTLResourceUsageRead];
      for (int k = 0; k < 6; k++) [ce setTexture:gtex[k] atIndex:k];
    };
    auto dispatch2D = [&](id<MTLComputeCommandEncoder> ce, id<MTLComputePipelineState> pso, int w, int h) {
      MTLSize tg = MTLSizeMake(pso.threadExecutionWidth, pso.maxTotalThreadsPerThreadgroup / pso.threadExecutionWidth, 1);
      [ce dispatchThreadgroups:MTLSizeMake((w + tg.width - 1) / tg.width, (h + tg.height - 1) / tg.height, 1) threadsPerThreadgroup:tg];
    };
    if (!noGI) {
      bool have = false;
      u32 key = pcg((u32)nt * 31u + (u32)nv) ^ pcg((u32)(sc.sun.x * 1e4f) + 77u) ^ (u32)(gx * 73 + gy * 31 + gz);
      if (!rebake) { FILE* f = fopen(giCache, "rb"); if (f) { u32 k2 = 0; fread(&k2, 4, 1, f); if (k2 == key && fread(gridE.data(), 4, gridE.size(), f) == gridE.size()) have = true; fclose(f); } }
      if (!have) {
        double tb = nowSec();
        Rig g0 = rigAt(sim, 0); buildTLAS(dinoXf(sim, sc.dinoAnim, 0), robotXf(g0));
        id<MTLBuffer> bG = [dev newBufferWithLength:nvox * 7 * 16 options:MTLResourceStorageModeShared];
        const int bounces = 4, passes = 12; U->grid[3] = 256;
        for (int b = 0; b < bounces; b++) {
          memset(bG.contents, 0, nvox * 7 * 16);
          for (int p = 0; p < passes; p++) {
            U->counts[0] = 1000 + b * 100 + p;
            id<MTLCommandBuffer> cb = [queue commandBuffer]; id<MTLComputeCommandEncoder> ce = [cb computeCommandEncoder];
            [ce setComputePipelineState:psoBake]; [ce setBuffer:bG offset:0 atIndex:0]; bindCommon(ce); dispatch2D(ce, psoBake, gx * gz, gy);
            [ce endEncoding]; [cb commit]; [cb waitUntilCompleted];
            if (cb.error) { fprintf(stderr, "bake error: %s\n", cb.error.localizedDescription.UTF8String); return 1; }
          }
          const float* ga = (const float*)bG.contents; std::vector<uint8_t> valid(nvox, 0);
          for (size_t v = 0; v < nvox; v++) {
            float ins = ga[(v * 7 + 6) * 4], cnt = ga[(v * 7 + 6) * 4 + 1]; valid[v] = cnt > 0 && ins / cnt < 0.30f;
            for (int k = 0; k < 6; k++) for (int c = 0; c < 3; c++) gridE[(v * 6 + k) * 3 + c] = valid[v] ? ga[(v * 7 + k) * 4 + c] * 4 * PI / cnt : 0.0f;
          }
          for (int it = 0; it < 8; it++) { // fill voxels that sit inside solids from their neighbours
            std::vector<uint8_t> nvld = valid; std::vector<float> ng = gridE;
            for (int z = 0; z < gz; z++) for (int y = 0; y < gy; y++) for (int x = 0; x < gx; x++) {
              size_t v = x + (size_t)gx * (y + (size_t)gy * z); if (valid[v]) continue;
              float a[18] = {0}; int n = 0; const int o[6][3] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
              for (int k = 0; k < 6; k++) { int xx = x + o[k][0], yy = y + o[k][1], zz = z + o[k][2]; if (xx < 0 || yy < 0 || zz < 0 || xx >= gx || yy >= gy || zz >= gz) continue;
                size_t w = xx + (size_t)gx * (yy + (size_t)gy * zz); if (!valid[w]) continue; for (int c = 0; c < 18; c++) a[c] += gridE[w * 18 + c]; n++; }
              if (n) { for (int c = 0; c < 18; c++) ng[v * 18 + c] = a[c] / n; nvld[v] = 1; }
            }
            valid.swap(nvld); gridE.swap(ng);
          }
          uploadGrid();
        }
        FILE* f = fopen(giCache, "wb"); if (f) { fwrite(&key, 4, 1, f); fwrite(gridE.data(), 4, gridE.size(), f); fclose(f); }
        fprintf(stderr, "bounce-light grid %dx%dx%d baked in %.1fs\n", gx, gy, gz, nowSec() - tb);
      } else { uploadGrid(); fprintf(stderr, "bounce-light grid loaded from %s\n", giCache); }
    }

    // ---- per-frame curves: focus (a lagging autofocus) and exposure (from the metering pass, lagging too)
    std::vector<float> focusF(NF + 2, 1.6f), expoF(NF + 2, fixedExpo > 0 ? fixedExpo : 1.0f);
    { float f = 1.6f; int sub = 8; for (int i = 0; i < NF + 2; i++) { for (int k = 0; k < sub; k++) { float tt = (i + (float)k / sub) / FPS; float tg = focusTarget(sim, tt); f += (tg - f) * (1 - expf(-(1.0f / (FPS * sub)) / 0.22f)); } focusF[i] = f; } }
    if (expoFile && fixedExpo <= 0) {
      FILE* f = fopen(expoFile, "r"); std::vector<std::pair<int, float>> mt; int fr; float lg;
      if (f) { while (fscanf(f, "%d %f", &fr, &lg) == 2) mt.push_back({fr, lg}); fclose(f); }
      if (mt.size() > 1) {
        float e = 0; bool first = true; size_t j = 0;
        for (int i = 0; i < NF + 2; i++) {
          while (j + 1 < mt.size() && mt[j + 1].first <= i) j++;
          float lgi = mt[j].second; if (j + 1 < mt.size() && mt[j + 1].first > mt[j].first) { float u = clampf((float)(i - mt[j].first) / (mt[j + 1].first - mt[j].first), 0, 1); lgi = lerpf(mt[j].second, mt[j + 1].second, u); }
          float target = logf(0.20f) - lgi * 0.86f - 0.14f * (-1.9f); // partial compensation: dark places stay a little dark
          target = clampf(target, logf(0.12f), logf(30.0f));
          if (first) { e = target; first = false; } else e += (target - e) * (1 - expf(-(1.0f / FPS) / 0.42f));
          expoF[i] = expf(e);
        }
      } else fprintf(stderr, "warning: no usable metering in %s, exposure fixed at 1\n", expoFile);
    }

    // ---- frame loop
    size_t npx = (size_t)W * H;
    id<MTLBuffer> bAcc = [dev newBufferWithLength:npx * 16 options:MTLResourceStorageModeShared], bA0 = [dev newBufferWithLength:npx * 16 options:MTLResourceStorageModeShared], bA1 = [dev newBufferWithLength:npx * 16 options:MTLResourceStorageModeShared];
    id<MTLBuffer> bAccE = [dev newBufferWithLength:npx * 16 options:MTLResourceStorageModeShared], bE0 = [dev newBufferWithLength:npx * 16 options:MTLResourceStorageModeShared], bE1 = [dev newBufferWithLength:npx * 16 options:MTLResourceStorageModeShared], bGd = [dev newBufferWithLength:npx * 16 options:MTLResourceStorageModeShared];
    id<MTLBuffer> bDP[8]; for (int k = 0; k < 8; k++) { u32 d[4] = {(u32)W, (u32)H, 1u << k, 0}; bDP[k] = [dev newBufferWithBytes:d length:16 options:MTLResourceStorageModeShared]; }
    std::vector<float> img;
    U->dims[0] = W; U->dims[1] = H; U->dims[2] = mode;
    FILE* pipe = nullptr; FILE* mf = meterOut ? fopen(meterOut, "w") : nullptr;
    if (segOut) { char cmd[1024]; snprintf(cmd, sizeof cmd, "ffmpeg -y -loglevel error -f rawvideo -pix_fmt rgb24 -s %dx%d -r %g -i - -c:v libx264 -preset medium -crf 13 -pix_fmt yuv420p -g 30 -colorspace bt709 -color_primaries bt709 -color_trc bt709 '%s'", W, H, FPS, segOut); pipe = popen(cmd, "w"); if (!pipe) { fprintf(stderr, "cannot start ffmpeg\n"); return 1; } }
    if (framesDir) mkdir(framesDir, 0755);
    std::vector<uint8_t> rgb; double tLoop = nowSec(); int done = 0;
    for (int fr = from; fr <= to; fr += step) {
      double tf = nowSec();
      float t0 = fr / FPS, t1 = t0 + 0.5f / FPS, tm = (t0 + t1) * 0.5f;
      Rig r0 = rigAt(sim, t0), r1 = rigAt(sim, t1), rm = rigAt(sim, tm);
      if (freeCam) { V3 f = norm(at - eye), r = norm(cross(f, V3(0, 1, 0))), u = cross(r, f); // same handedness as the robot camera
        r0.eye = r1.eye = eye; r0.cF = r1.cF = f; r0.cR = r1.cR = r; r0.cU = r1.cU = u; tm = tFree; rm = rigAt(sim, tFree); }
      set3(U->camP0, r0.eye); set3(U->camP1, r1.eye); set3(U->camR0, r0.cR); set3(U->camU0, r0.cU); set3(U->camF0, r0.cF); set3(U->camR1, r1.cR); set3(U->camU1, r1.cU); set3(U->camF1, r1.cF);
      float foc = focusOv > 0 ? focusOv : focusF[std::min(fr, NF + 1)];
      U->lens[0] = tanf(rad(hfov) / 4); U->lens[1] = aper; U->lens[2] = foc; U->lens[3] = (float)H / W; U->misc[0] = tm;
      U->misc[1] = clampI / fmaxf(expoF[std::min(fr, NF + 1)], 0.05f); // firefly clamp relative to what the exposure makes white
      buildTLAS(dinoXf(sim, sc.dinoAnim, tm), robotXf(rm));
      memset(bAcc.contents, 0, npx * 16); memset(bA0.contents, 0, npx * 16); memset(bA1.contents, 0, npx * 16); memset(bAccE.contents, 0, npx * 16);
      int left = spp, pass = 0;
      while (left > 0) {
        int n = std::min(left, sppPass); U->counts[0] = (u32)fr * 131u + pass * 7u + 1u; U->counts[2] = n;
        id<MTLCommandBuffer> cb = [queue commandBuffer]; id<MTLComputeCommandEncoder> ce = [cb computeCommandEncoder];
        [ce setComputePipelineState:psoRender]; [ce setBuffer:bAcc offset:0 atIndex:0]; bindCommon(ce); [ce setBuffer:bA0 offset:0 atIndex:10]; [ce setBuffer:bA1 offset:0 atIndex:11]; [ce setBuffer:bAccE offset:0 atIndex:12];
        dispatch2D(ce, psoRender, W, H); [ce endEncoding]; [cb commit]; [cb waitUntilCompleted];
        if (cb.error) { fprintf(stderr, "render error at frame %d: %s\n", fr, cb.error.localizedDescription.UTF8String); return 1; }
        left -= n; pass++;
      }
      const float* acc = (const float*)bAcc.contents;
      { // soft light: normalise, then filter (dnIter passes with growing step)
        id<MTLCommandBuffer> cb = [queue commandBuffer]; id<MTLComputeCommandEncoder> ce = [cb computeCommandEncoder];
        [ce setComputePipelineState:psoPrep]; [ce setBuffer:bAccE offset:0 atIndex:0]; [ce setBuffer:bA0 offset:0 atIndex:1]; [ce setBuffer:bA1 offset:0 atIndex:2]; [ce setBuffer:bAcc offset:0 atIndex:3];
        [ce setBuffer:bE0 offset:0 atIndex:4]; [ce setBuffer:bGd offset:0 atIndex:5]; [ce setBuffer:bDP[0] offset:0 atIndex:6]; dispatch2D(ce, psoPrep, W, H);
        [ce setComputePipelineState:psoAtrous]; [ce setBuffer:bGd offset:0 atIndex:1];
        for (int k = 0; k < dnIter; k++) { [ce setBuffer:(k & 1) ? bE1 : bE0 offset:0 atIndex:0]; [ce setBuffer:(k & 1) ? bE0 : bE1 offset:0 atIndex:2]; [ce setBuffer:bDP[k] offset:0 atIndex:3]; dispatch2D(ce, psoAtrous, W, H); }
        [ce endEncoding]; [cb commit]; [cb waitUntilCompleted];
        if (cb.error) { fprintf(stderr, "filter error at frame %d: %s\n", fr, cb.error.localizedDescription.UTF8String); return 1; }
      }
      combine(acc, (const float*)bA0.contents, (const float*)((dnIter & 1) ? bE1 : bE0).contents, W, H, img);
      if (mf) { // centre-weighted log-average luminance
        double s = 0, w = 0;
        for (int y = 0; y < H; y++) for (int x = 0; x < W; x++) { const float* p = &img[((size_t)y * W + x) * 3]; float l = 0.2126f * p[0] + 0.7152f * p[1] + 0.0722f * p[2];
          float nx = (x + 0.5f) / W * 2 - 1, ny = (y + 0.5f) / H * 2 - 1; float wt = expf(-(nx * nx + ny * ny * 1.4f) * 1.1f) * (ny > 0.62f ? 0.25f : 1.0f);
          s += wt * logf(clampf(l, 2e-3f, 12.0f)); w += wt; }
        fprintf(mf, "%d %.5f\n", fr, s / w); fflush(mf);
      } else {
        PostP P; P.exposure = expoF[std::min(fr, NF + 1)]; P.seed = (u32)fr; P.raw = raw || mode != 0;
        float tt = fr / FPS, dur = NF / FPS;
        P.fade = freeCam ? 1.0f : smooth(0.30f, 1.35f, tt) * smooth(dur - 0.05f, dur - 1.5f, tt);
        P.grain = clampf(0.75f + 0.35f * logf(fmaxf(P.exposure, 0.2f)), 0.6f, 1.9f);
        postProcess(img, W, H, P, rgb);
        if (pipe) { if (fwrite(rgb.data(), 1, rgb.size(), pipe) != rgb.size()) { fprintf(stderr, "ffmpeg pipe closed\n"); return 1; } }
        if (framesDir) { char fn[512]; snprintf(fn, sizeof fn, "%s/f_%05d.ppm", framesDir, fr); FILE* f = fopen(fn, "wb"); fprintf(f, "P6\n%d %d\n255\n", W, H); fwrite(rgb.data(), 1, rgb.size(), f); fclose(f); }
      }
      done++;
      if (done <= 3 || done % 20 == 0) fprintf(stderr, "frame %d  %.2fs  (avg %.2fs/frame, exposure %.2f, focus %.2f)\n", fr, nowSec() - tf, (nowSec() - tLoop) / done, expoF[std::min(fr, NF + 1)], foc);
    }
    if (pipe) { int rc = pclose(pipe); if (rc != 0) { fprintf(stderr, "ffmpeg failed\n"); return 3; } }
    if (mf) fclose(mf);
    fprintf(stderr, "done: %d frames in %.1fs\n", done, nowSec() - tStart);
  }
  return 0;
}
