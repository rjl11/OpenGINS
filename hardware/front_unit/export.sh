#!/usr/bin/env bash
# Export the printable parts (stl/) and the pictures (img/).
#   ./export.sh            (OPENSCAD=/path/to/openscad to override)
set -eu
cd "$(dirname "$0")"
OPENSCAD=${OPENSCAD:-openscad}
command -v "$OPENSCAD" >/dev/null 2>&1 || OPENSCAD="/c/Program Files/OpenSCAD/openscad.com"
mkdir -p stl img

for p in base lid tap_test; do
    out=$("$OPENSCAD" -o "stl/$p.stl" -D "part=\"$p\"" front_unit.scad 2>&1)
    echo "$out" | grep -E "^ECHO" | sort -u || true
    if echo "$out" | grep -q "Simple: *yes"; then echo "stl/$p.stl ok"
    else echo "stl/$p.stl NOT a clean solid"; echo "$out" | grep -iE "warn|error"; exit 1; fi
done

# --camera=eye x,y,z,centre x,y,z
img() {
    "$OPENSCAD" -o "img/$1.png" --imgsize=1400,1000 --colorscheme=Tomorrow \
        --camera="$2" ${4:-} -D "part=\"$3\"" front_unit.scad >/dev/null 2>&1
    echo "img/$1.png"
}
img assembly "-110,-160,170,55,34,25" assembly
img open     "-90,-130,200,55,34,15" open
img bottom   "-90,-150,-150,55,34,20" assembly
img section  "55,-260,25,55,34,25" section --projection=o
