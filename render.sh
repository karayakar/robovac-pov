#!/bin/bash
# Full pipeline: build -> timeline -> metering pre-pass -> segmented render (resumable) -> sound -> mux.
# usage: ./render.sh [out.mp4] [spp]
set -e
cd "$(dirname "$0")"
OUT=${1:-renders/v1.mp4}; SPP=${2:-36}; SEG=100
./build.sh
mkdir -p work/seg renders
[ -x .venv/bin/python ] || { python3 -m venv .venv && .venv/bin/pip install -q numpy scipy; }
./robovac --timeline work/timeline.json
FRAMES=$(python3 -c "import json;print(json.load(open('work/timeline.json'))['frames'])")
# 1. what would the camera's meter see? (small, fast) -> drives the lagging auto-exposure
[ -s work/meter.txt ] || ./robovac --w 480 --h 270 --spp 8 --step 2 --meter work/meter.txt
# 2. the film, in segments so an interrupted render can pick up where it stopped
i=0
for ((a = 0; a < FRAMES; a += SEG)); do
  b=$((a + SEG - 1)); [ $b -ge $FRAMES ] && b=$((FRAMES - 1))
  f=$(printf "work/seg/s_%03d.mp4" $i)
  if [ ! -s "$f" ]; then
    ./robovac --spp "$SPP" --from $a --to $b --expo work/meter.txt --seg "work/seg/tmp.mp4" && mv work/seg/tmp.mp4 "$f"
  fi
  i=$((i + 1))
done
ls work/seg/s_*.mp4 | sed "s|^|file '$PWD/|; s|$|'|" > work/seg/list.txt
ffmpeg -y -loglevel error -f concat -safe 0 -i work/seg/list.txt -c copy work/video.mp4
# 3. sound, synthesised from the event timeline
.venv/bin/python src/audio.py work/timeline.json work/audio.wav
# 4. one final encode + mux
ffmpeg -y -loglevel error -i work/video.mp4 -i work/audio.wav -c:v libx264 -preset slow -crf 22 -pix_fmt yuv420p \
  -colorspace bt709 -color_primaries bt709 -color_trc bt709 -c:a aac -b:a 256k -movflags +faststart -shortest "$OUT"
echo "wrote $OUT"
