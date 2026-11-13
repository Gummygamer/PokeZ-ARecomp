#!/bin/bash
# Windows (Git Bash): Pokémon Z-A in the headset (docs/vr.md), from a dev build. Connect the headset first (Quest:
# Link / Air Link). A normal window: the keyboard, a gamepad and the headset's controllers all play.
#   ROOT=<data folder> with extracted/ (zaimport's output), save/save_0 and larecomp.ini (default: testroot; the ini
#   is made from za-default.ini the first time)
#   EXE=<lasrecomp.exe> with its DLLs next to it (default: build/dist-za/ZA Recomp/lasrecomp.exe)
#   LIB=<lasgame.dll> (default: build/za-dist/lasgame.dll - the library is not copied into extracted/)
#   VR=0: on the window only (the same build without the headset)
#   FPS=30: the game's own 30 frames a second (default 60: half the wait between the head's move and the picture's)
# Log: <ROOT>/play.log - "[vr]" the session and the throws, "[ball]" every velocity the game gives a ball
# (LARECOMP_LOG_BALL: which throws go through the patched node, and how fast - tools/za_patches.py ZA_BALL).
cd "$(dirname "$0")/../.." || exit 1
R=${ROOT:-testroot}
EXE=${EXE:-build/dist-za/ZA Recomp/lasrecomp.exe}
LIB=${LIB:-build/za-dist/lasgame.dll}
[ -f "$R/extracted/romfs.bin" ] || { echo "no game files in $R/extracted (zaimport: docs/linux.md, Game files)"; exit 1; }
[ -f "$EXE" ] || { echo "no $EXE (CLAUDE.md: Building)"; exit 1; }
[ -f "$LIB" ] || { echo "no game code library at $LIB (make ... lasgame.dll)"; exit 1; }
[ -f "$R/larecomp.ini" ] || cp tools/za/za-default.ini "$R/larecomp.ini"
mkdir -p "$R/save"
# (full Windows paths: the game runs in the data root and knows no /d/... paths)
LARECOMP_VR=${VR:-1} LARECOMP_FPS=${FPS:-60} LARECOMP_LOG_BALL=1 LARECOMP_GAME_LIB="$(cygpath -aw "$LIB")" LARECOMP_DATA_ROOT="$(cygpath -aw "$R")" \
    LARECOMP_SETTINGS="$(cygpath -aw "$R/larecomp.ini")" "$EXE" "$(cygpath -w "$R/extracted")" --play > "$R/play.log" 2>&1
echo "rc=$?"
grep -a "^\[vr\]" "$R/play.log" | head -20
echo "[ball] lines: $(grep -ac "^\[ball\]" "$R/play.log")"
