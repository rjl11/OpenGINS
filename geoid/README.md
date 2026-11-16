# Geoid model: SWEN17_RH2000

Lantmäteriet's geoid model SWEN17_RH2000, from
<https://www.lantmateriet.se/en/geodata/gps-geodesy-and-swepos/transformations/>
(section "SWEN17_RH2000", file `swen17_rh2000_txt.zip`).
It is published as open data under CC0 1.0.

## What it is

The model gives the geoid height N above the GRS80 ellipsoid. It converts an
ellipsoid height in **SWEREF 99** to a height above sea level in **RH 2000**:
`H = h - N`. It is a gravimetric geoid fitted to GNSS / levelling points. It
also includes the land uplift between the SWEREF 99 epoch (1999.5) and the
RH 2000 epoch (2000.0).

Lantmäteriet gives a standard uncertainty of 8–10 mm over the Swedish mainland,
Öland and Gotland. It is larger near Vättern and the northern border with Norway
(2–3 cm), in the high north-west mountains (2–4 cm), in coastal waters (2–3 cm)
and further out at sea (probably 5–10 cm).

The result is an RH 2000 height only if `h` is in SWEREF 99. For this unit, that
means an RTK fix from the base, whose coordinates are in SWEREF 99. A fix from
the receiver alone is in WGS84 / ITRF and is a few decimetres off SWEREF 99.

## File layout

`SWEN17_RH2000.txt` is an ASCII grid in the GRAVSOFT layout, and it is the source for the firmware table:
- **Header line:** `lat_south lat_north lon_west lon_east dlat dlon` in degrees. Here
  it is `54 70 10 25 0.01 0.02`, so the grid is 1601 rows × 751 columns.
- **Heights:** N in metres, row by row from the **north** edge southwards. Each row
  runs from west to east and is wrapped 8 values per line, with a blank line
  between rows.
- **Range:** N is 17.3–43.2 m over the whole grid.

## How the firmware uses it

The whole grid is too big for the Pico's flash, so `firmware/tools/geoid_cut.py`
cuts out the area around the RTK base and writes it as int16 millimetres to
`firmware/app/geoid_grid.c`. The current cut is 100 km around the base at
58N 12E, at full resolution: 182 × 172 points, 61 KB.

The code in `firmware/app/geoid.c` interpolates bilinearly between the four
nearest grid points. Lantmäteriet names bicubic splines as one suitable method.
They also say bilinear interpolation on the four nearest nodes gives a slightly
larger error, but one that is very small next to the model's uncertainty.

Outside the cut, the firmware falls back to the receiver's own geoid. The field
`geoid_model` in `/status.json` says which one was used.

To make the table again, for example for another base or radius, run this from the repo root:

```bash
python firmware/tools/geoid_cut.py geoid/SWEN17_RH2000.txt --lat <base latitude> --lon <base longitude> --radius-km 100 -o firmware/app/geoid_grid.c
```

Then check the table against the source with `firmware/tools/test_geoid.c`. The
build command is at the top of that file. Pass `geoid/SWEN17_RH2000.txt` as its
argument.

SWEN17_RH70, for the older RH 70 height system, is on the same page and is not
included here.
