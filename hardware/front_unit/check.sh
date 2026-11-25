#!/usr/bin/env bash
# Clearance check: every printed part against every bought part (and the
# bought parts against each other) must not overlap. Each pair renders the
# intersection; OpenSCAD reports an empty top-level object when they don't.
# The layout asserts in front_unit.scad (FPC length, TF-Luna opening) run too.
#
#   ./check.sh            (OPENSCAD=/path/to/openscad to override)
set -u
cd "$(dirname "$0")"
OPENSCAD=${OPENSCAD:-openscad}
command -v "$OPENSCAD" >/dev/null 2>&1 || OPENSCAD="/c/Program Files/OpenSCAD/openscad.com"

pairs="base:cam base:tm171 base:tfluna base:rock lid:rock
cam:rock cam:tm171 cam:tfluna tfluna:rock tm171:rock"
tmp=$(mktemp -d)
fail=0
for p in $pairs; do
    a=${p%%:*}; b=${p##*:}
    out=$("$OPENSCAD" -o "$tmp/$a-$b.stl" -D "part=\"check\"" -D "check_a=\"$a\"" -D "check_b=\"$b\"" front_unit.scad 2>&1)
    if echo "$out" | grep -q "Assertion"; then
        echo "FAIL $a / $b: assertion"; echo "$out" | grep Assertion; fail=1
    elif echo "$out" | grep -q "top level object is empty"; then
        echo "ok   $a / $b"
    else
        echo "FAIL $a / $b overlap"; fail=1
    fi
done
rm -rf "$tmp"
exit $fail
