# Prompt

A self-contained prompt for making this kind of video from nothing, with no reference project to look at.

This is not the request that produced the video in this repository. That request pointed at an existing project (see the README). This prompt was written afterwards from what turned out to matter, and it has not been run yet. A fresh run will build its own room and its own look; for the exact video, build this repository.

---

Create a 50 to 60 second video that looks like real footage and is made entirely with code.

## The shot

A small action camera is taped to the top of a robot vacuum. The video is one continuous shot from that camera while the robot does an afternoon cleaning run through a sunlit living room:

1. It wakes up on its dock and drives out across an oak plank floor, past a few toys.
2. It climbs onto a rug and passes under a coffee table.
3. It goes under a sofa, where it is dark and dusty and lost things are lying around (a sock, a coin, a crayon, dust bunnies).
4. It comes out, drives along a bookshelf, then under a dining table and over some spilled breakfast cereal.
5. It bumps into a toy T-Rex standing in a patch of sun by the window. The toy gets shoved, rocks on its feet, and tips over onto its side.
6. The robot waits a moment, reverses, turns away and drives off. Fade out.

The front edge of the robot's own glossy black deck is visible along the bottom of the frame the whole time, with dust on it.

## Rules

- No AI image or video models.
- No Blender, game engine or other 3D software.
- No downloaded or bundled assets of any kind: no textures, 3D models, photos, footage, fonts or sound recordings.
- Every pixel and every audio sample comes from code you write in this session. ffmpeg may only encode the frames and mux the audio.
- Output: 1920x1080, 30 fps, H.264 mp4 with stereo sound.

## What makes it read as footage

People decide "real" from light and camera behaviour long before they look at object detail. Spend the effort there.

- **Light.** Write a physically based path tracer. Low afternoon sun through tall windows, soft shadow edges, sky light through the window openings, and bounce light off the sunlit floor filling the room. The bounce light is what makes an interior look photographed.
- **Camera.** Simulate a physical device, not a viewpoint: a wide-angle lens with real distortion, a small aperture with depth of field, a 180 degree shutter with motion blur, auto-exposure that lags when the robot goes from sun into shadow and back, autofocus that takes a moment to find something close, glare around bright windows, a little chromatic aberration, vignetting, and sensor grain that gets worse in the dark.
- **Motion.** The camera inherits the robot's motion: limited acceleration, motor vibration, a jolt when a wheel climbs the rug edge, small ticks crossing floorboard seams, a hard stop at the bump.
- **Geometry.** Real-world units. A bevel on every edge. Variation everywhere: plank lengths and tones, book sizes, leaning objects. The rug is individual fibres, not a flat pattern, because the camera is a few centimetres above it.
- **Materials.** All procedural: wood grain, woven fabric, wall paint with a slight orange peel, glossy plastic, ceramic, leaves that let light through.
- **Sound.** Synthesise it from an event timeline that the same simulation exports, so every sound lines up with the picture: suction fan and drive motors that change pitch on carpet and in turns, wheel ticks on the seams, a thump at the rug edge, cereal crunching in the brush, the bumper clack, the hollow toy hitting the floor. Everything gets boxy and muffled under the furniture. The microphone is on the robot, so the robot is always the loudest thing.

## What has worked for the engine

Use this unless you find something better on this machine (an Apple Silicon Mac with the command line tools, ffmpeg and Python 3).

- Scene and robot simulation in C++. One mesh toolkit: rounded boxes, lathes, tubes, splines, and a signed-distance mesher for organic shapes like the toy.
- Renderer as a Metal compute shader compiled at run time, using the GPU's hardware ray tracing. No Xcode project needed.
- To keep the render to about an hour: bake the multi-bounce light into a coarse 3D grid once, trace one real bounce per sample, and denoise only the soft indirect light using normals and depth as a guide. Sun, highlights and textures stay unfiltered and sharp. Around 36 samples per pixel is then enough.
- Lens and sensor effects as a post step on the linear image, before tone mapping.
- A metering pre-pass at low resolution to drive the auto-exposure curve.
- Sound in Python with numpy and scipy.

## How to work

- Before the full render, render stills along the whole route and look at them yourself. Fix whatever looks fake, then look again. Check the toy close up from a free camera, because it fills the frame at the climax.
- Add debug views (surface colour, normals, direct light only, indirect light only) and use them when something looks wrong instead of guessing.
- Check the route against the furniture so the robot never drives through anything.
- Render in segments that can resume after an interruption.
- Keep the full render under 90 minutes on this machine.
- Afterwards: contact sheet of the whole video, ffprobe, black and frozen frame detection, loudness measurement.

Carry this through to the finished mp4 without asking me questions; the design decisions are yours.

## Deliver

- The mp4, a contact sheet and a few stills.
- The source, with one script that rebuilds everything.
- A short report: how it works, what you checked, what you did not check, and what still looks wrong.
