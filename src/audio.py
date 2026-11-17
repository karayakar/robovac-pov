#!/usr/bin/env python3
"""Sound track for the robot-vacuum film.

Everything is synthesised with numpy/scipy from the renderer's event timeline (work/timeline.json):
no samples, no recordings. The "microphone" sits on the robot next to the camera, so the fan and the
drive motors are always the loudest thing, and the room only shows up as short reflections.

usage: audio.py timeline.json out.wav
"""
import json
import sys

import numpy as np
from scipy import signal
from scipy.io import wavfile

SR = 48000
rng = np.random.default_rng(7)


# ---------------------------------------------------------------- small helpers
def filt(x, kind, f, order=2):
    return signal.sosfilt(signal.butter(order, f, btype=kind, fs=SR, output="sos"), x, axis=0)


def lp(x, f, o=2):
    return filt(x, "lowpass", f, o)


def hp(x, f, o=2):
    return filt(x, "highpass", f, o)


def bp(x, f0, f1, o=2):
    return filt(x, "bandpass", [f0, f1], o)


def noise(n):
    return rng.standard_normal(n)


def pink(n):
    b = [0.049922035, -0.095993537, 0.050612699, -0.004408786]
    a = [1, -2.494956002, 2.017265875, -0.522189400]
    return signal.lfilter(b, a, noise(n)) * 6.0


def smooth(x, tau):
    a = 1 - np.exp(-1 / (tau * SR))
    return signal.lfilter([a], [1, -(1 - a)], x)


def tone(freq, amp=1.0, phase=0.0):
    """sine with a time-varying frequency (array, Hz)"""
    return amp * np.sin(2 * np.pi * np.cumsum(freq) / SR + phase)


def ping(f, tau, dur=None):
    n = int((dur or tau * 8) * SR)
    t = np.arange(n) / SR
    return np.sin(2 * np.pi * f * t) * np.exp(-t / tau)


def burst(dur, f0, f1, tau, attack=0.0004):
    n = int(dur * SR)
    t = np.arange(n) / SR
    return bp(noise(n), f0, f1) * np.exp(-t / tau) * np.minimum(t / attack, 1.0)


def place(buf, t, s, gain=1.0, pan=0.0):
    i = int(round(t * SR))
    if i < 0 or i >= len(buf):
        return
    n = min(len(s), len(buf) - i)
    a = (pan + 1) * np.pi / 4
    buf[i:i + n, 0] += s[:n] * gain * np.cos(a) * 1.414
    buf[i:i + n, 1] += s[:n] * gain * np.sin(a) * 1.414


def add_at(out, i, s, g=1.0):
    n = min(len(s), len(out) - i)
    if n > 0:
        out[i:i + n] += s[:n] * g


def mix_into(dst, *parts):
    n = max(len(p) for p in parts)
    out = np.zeros(n)
    for p in parts:
        out[:len(p)] += p
    return out


# ---------------------------------------------------------------- one-shot sounds
def chime():
    """three rising notes from a tiny piezo speaker"""
    out = np.zeros(int(0.75 * SR))
    for k, (f, t0, d) in enumerate([(1046.5, 0.0, 0.10), (1318.5, 0.11, 0.10), (1568.0, 0.22, 0.30)]):
        n = int(d * SR)
        t = np.arange(n) / SR
        env = np.minimum(t / 0.004, 1.0) * np.exp(-t / (d * 0.45))
        s = (np.sin(2 * np.pi * f * t) + 0.35 * np.sin(2 * np.pi * 2 * f * t) + 0.18 * np.sin(2 * np.pi * 3 * f * t)) * env
        add_at(out, int(t0 * SR), s)
    return bp(out, 700, 6000)


def seam_tick(a):
    """a wheel dropping over the joint between two floorboards"""
    s = mix_into(None, burst(0.03, 1200, 4200, 0.0035) * 0.55, ping(118 + 30 * rng.random(), 0.016) * 0.9, burst(0.05, 250, 700, 0.010) * 0.5)
    return s * a


def rug_thump(strength):
    return mix_into(None, ping(74, 0.045) * 1.0, burst(0.12, 120, 500, 0.03) * 0.7, burst(0.03, 900, 2500, 0.006) * 0.15) * strength


def cereal_crunch():
    """a loop of cereal is flicked by the side brush, crunched by the roller and rattles into the bin"""
    out = np.zeros(int(0.5 * SR))
    t = 0.0
    for k in range(rng.integers(4, 8)):
        f = rng.uniform(1700, 4200)
        s = mix_into(None, ping(f, rng.uniform(0.003, 0.008)) * rng.uniform(0.4, 1.0), burst(0.02, 2500, 8000, 0.004) * rng.uniform(0.3, 0.9))
        add_at(out, int(t * SR), s, 0.85 ** k)
        t += rng.uniform(0.012, 0.07)
    crunch = burst(0.09, 1500, 6000, 0.02) * 0.8  # the crunch itself
    add_at(out, 0, crunch)
    return out


def bumper_clack():
    out = np.zeros(int(0.6 * SR))
    for t0, g in [(0.0, 1.0), (0.021, 0.45)]:  # the bumper closing, then springing back
        s = mix_into(None, ping(930, 0.012) * 0.7, ping(2350, 0.006) * 0.5, ping(410, 0.02) * 0.6, burst(0.03, 1500, 7000, 0.004) * 0.8)
        add_at(out, int(t0 * SR), s, g)
    thud = mix_into(None, ping(236, 0.05) * 0.9, ping(118, 0.06) * 0.6, burst(0.12, 150, 600, 0.03) * 0.5)  # the hollow toy taking the hit
    add_at(out, 0, thud)
    return out


def toy_tap(strength, pitch=1.0):
    """hollow vinyl toy touching a wooden floor"""
    s = mix_into(None, ping(330 * pitch, 0.035) * 0.8, ping(655 * pitch, 0.022) * 0.5, ping(1420 * pitch, 0.010) * 0.35,
                 burst(0.05, 700, 5000, 0.006) * 0.6, ping(92, 0.05) * 0.5)
    return s * strength


def reverb_ir(rt60=0.42, dur=0.6):
    n = int(dur * SR)
    t = np.arange(n) / SR
    ir = np.stack([lp(noise(n), 5200), lp(noise(n), 5200)], axis=1) * np.exp(-6.9 * t / rt60)[:, None]
    ir[:int(0.004 * SR)] = 0  # a few ms before the first reflection (walls are metres away)
    for d, g in [(0.0052, 0.5), (0.0091, 0.38), (0.0137, 0.3)]:  # floor / furniture early reflections
        ir[int(d * SR)] += g * 14
    return ir / np.sqrt((ir ** 2).sum(axis=0)).max()


# ---------------------------------------------------------------- main
def main():
    tl = json.load(open(sys.argv[1]))
    out_path = sys.argv[2]
    dur = tl["duration"]
    N = int(dur * SR)
    ta = np.arange(N) / SR
    st = np.array(tl["state"])
    ts = np.arange(len(st)) / 120.0

    def S(k):
        return np.interp(ta, ts, st[:, k])

    v, om, vib, rug, sofa, table = (S(k) for k in range(6))
    ev = tl["events"]

    def T(name):
        return [e for e in ev if e["type"] == name]

    t_on = T("motor_on")[0]["t"]
    spin = np.where(ta > t_on, 1 - np.exp(-(ta - t_on) / 0.38), 0.0)
    rug_s = smooth(rug, 0.30)

    # ---- suction fan: rotor hum, blade-pass whine, and a lot of air
    wob = 1 + 0.0025 * lp(noise(N), 6) * 8 + 0.0012 * np.sin(2 * np.pi * 0.31 * ta)
    f_rot = 168.0 * spin ** 0.75 * (1 + 0.075 * rug_s) * wob  # carpet boost: the fan speeds up on the rug
    fan_tone = (tone(f_rot, 0.9) + tone(2 * f_rot, 0.5, 1.0) + tone(3 * f_rot, 0.22, 2.0) + tone(7 * f_rot, 0.50, 0.3)
                + tone(14 * f_rot, 0.16, 1.7) + tone(21.03 * f_rot, 0.05, 0.5)) * spin ** 2
    air_gain = spin ** 2 * (1 + 0.60 * rug_s)

    def air():
        p = pink(N)
        return (bp(p, 320, 1900) * 1.0 + bp(p, 1900, 7500) * 0.55 + lp(p, 320) * 0.35) * air_gain

    air_c, air_l, air_r = air(), air(), air()

    # ---- brush roll: a soft flutter in the air noise, heavier on the rug
    f_br = 22.0 * spin * (1 - 0.12 * rug_s)
    flut = 0.5 + 0.5 * np.sin(2 * np.pi * np.cumsum(f_br) / SR)
    brush = bp(noise(N), 450, 2600) * (0.25 + 0.75 * flut ** 2) * spin * (0.55 + 0.5 * rug_s) + lp(noise(N), 380) * flut ** 3 * rug_s * spin * 1.6

    # ---- drive: gear whine follows the wheel speed, rolling noise follows the floor
    speed = smooth(np.abs(v) / 0.32, 0.02)
    turn = smooth(np.abs(om) / 1.36, 0.02)
    drive = np.clip(np.maximum(speed, 0.85 * turn), 0, 1.2)
    f_g = (88 + 640 * drive) * (1 + 0.004 * lp(noise(N), 12) * 6)
    gear = (tone(f_g, 1.0) + tone(2 * f_g, 0.55, 0.7) + tone(3.01 * f_g, 0.30, 1.9) + tone(5.98 * f_g, 0.12, 0.2)) * np.clip(drive, 0, 1) ** 1.3
    gear *= 1 + 0.25 * lp(noise(N), 30) * 3
    roll = bp(pink(N), 140, 1500) * speed * (1 - 0.65 * rug_s) + lp(noise(N), 650) * speed * rug_s * 0.9
    rattle = bp(noise(N), 2400, 6500) * np.clip(smooth(vib, 0.05), 0, 2) * (0.5 + 0.5 * np.abs(lp(noise(N), 9)) * 6)

    robot_m = 0.023 * fan_tone + 0.050 * air_c + 0.030 * brush + 0.027 * gear + 0.030 * roll + 0.006 * rattle
    robot = np.stack([robot_m + 0.032 * air_l, robot_m + 0.032 * air_r], axis=1)

    # ---- events
    evb = np.zeros((N + SR, 2))  # reverberant bus (things happening in the room)
    dryb = np.zeros((N + SR, 2))  # things happening on the robot itself
    place(evb, T("boot")[0]["t"], chime(), 0.13)
    for e in T("seam"):
        place(dryb, e["t"], seam_tick(e["a"]), 0.13, rng.uniform(-0.25, 0.25))
    for name, g in [("rug_on", 0.22), ("rug_on2", 0.20), ("rug_off", 0.26), ("rug_off2", 0.24)]:
        for e in T(name):
            place(dryb, e["t"], rug_thump(1.0), g)
    for e in T("cereal"):
        place(dryb, e["t"] + rng.uniform(0, 0.03), cereal_crunch(), 0.36, rng.uniform(-0.3, 0.3))
    tb = tl["bump"]
    place(dryb, tb, bumper_clack(), 0.42)
    place(evb, tb + 0.004, toy_tap(0.5, 0.9), 0.30, 0.15)
    for k in range(1, 5):  # it rocks on its feet before going over
        tk = k / (2 * 3.1)
        if tk < 0.78:
            place(evb, tb + tk, toy_tap(np.exp(-3.2 * tk), 1.0 + 0.06 * rng.standard_normal()), 0.16, 0.2)
    tf = T("dino_fall")[0]["t"]
    place(evb, tf, toy_tap(1.0, 0.82), 0.50, 0.3)
    place(evb, tf + 0.006, burst(0.12, 90, 400, 0.035), 0.4, 0.3)
    for k in range(1, 4):  # two or three small bounces
        tr = k / (2 * 4.2)
        place(evb, tf + tr, toy_tap(np.exp(-9.0 * tr) * 0.9, 0.9 + 0.1 * k), 0.5, 0.3)

    ir = reverb_ir()
    wet = np.stack([signal.fftconvolve(evb[:, c], ir[:, c])[:N + SR] for c in range(2)], axis=1)
    events = (evb * 0.85 + wet * 0.55 + dryb)[:N]
    # a little of the robot's own noise comes back from the room as well
    rwet = np.stack([signal.fftconvolve(robot[:, c], ir[:, c])[:N] for c in range(2)], axis=1)
    sig = robot + 0.10 * rwet + events

    # ---- under the sofa / the table the sound closes in: boxy, less top end, a short comb from the surface just above
    w = smooth(np.clip(0.9 * sofa + 0.32 * table, 0, 1), 0.10)[:, None]
    d = int(0.00105 * SR)
    comb = sig.copy()
    comb[d:] += 0.5 * sig[:-d]
    boxed = lp(comb, 2100) * 1.05 + bp(comb, 180, 420) * 0.5
    sig = sig * (1 - w) + boxed * w

    # ---- the room itself: air, and a city a long way behind double glazing
    amb = np.stack([lp(pink(N), 900), lp(pink(N), 900)], axis=1) * 0.0035
    city = np.stack([lp(noise(N), 170), lp(noise(N), 170)], axis=1) * (0.6 + 0.4 * np.sin(2 * np.pi * 0.07 * ta + 1.0))[:, None] * 0.05
    sig += (amb + city) * (1 - 0.6 * w)

    # ---- "microphone": rumble filter, fades with the picture, gentle limiter
    sig = hp(sig, 55)
    fade = np.clip((ta - 0.05) / 0.25, 0, 1) * np.clip((dur - 0.05 - ta) / 1.5, 0, 1) ** 1.2
    sig *= fade[:, None]
    gain = 2.0
    sig = np.tanh(sig * gain * 1.2) / 1.2
    peak = np.abs(sig).max()
    bed = np.sqrt((sig[int(20 * SR):int(30 * SR)] ** 2).mean())
    print(f"audio: {dur:.2f} s, peak {20 * np.log10(peak):.1f} dBFS, bed (20-30 s) {20 * np.log10(bed):.1f} dBFS RMS")
    wavfile.write(out_path, SR, (np.clip(sig, -1, 1) * 32767).astype(np.int16))


if __name__ == "__main__":
    main()
