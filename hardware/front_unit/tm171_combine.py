"""Combine SYD Dynamics' TM171 core and baseboard STEP models into one.

The two files come in unrelated coordinates. Both carry the four core
screw holes (25.6 x 21.4 mm), so the core is moved until its holes sit on
the baseboard's, with its underside on the board's top (z = 1.6).

Result, in the baseboard's frame (origin at the board centre, z up, pins at
+y): baseboard 34 x 40 x 1.6, core 30 x 26 x 11 on top, 12.6 overall. The
TM171's own axes (datasheet p.7): x = +y here (towards the pins),
y = -x here, z = +z.

Needs OCP (pip install cadquery-ocp). From the repo root:
    python -I hardware/front_unit/tm171_combine.py external/syd_dynamics
writes TransducerM_TM171_Combined.step and .stl into that folder and prints
the hole pattern found.
"""
import math
import os
import sys

from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.gp import gp_Ax1
from OCP.BRep import BRep_Builder
from OCP.BRepAlgoAPI import BRepAlgoAPI_Section
from OCP.BRepAdaptor import BRepAdaptor_Curve
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_AsIs, STEPControl_Reader, STEPControl_Writer
from OCP.StlAPI import StlAPI_Writer
from OCP.TopAbs import TopAbs_EDGE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS, TopoDS_Compound
from OCP.gp import gp_Dir, gp_Pln, gp_Pnt, gp_Trsf, gp_Vec

CORE = "TransducerM_TM171_TM151_Core_3D-Shape_V1-1-2.stp"
BOARD = "TransducerM_TM171_TM151_BaseBoard_3D-Shape_V1-1-4.stp"
OUT = "TransducerM_TM171_Combined"


def load(path):
    r = STEPControl_Reader()
    if r.ReadFile(path) != IFSelect_RetDone:
        sys.exit("can't read " + path)
    r.TransferRoots()
    return r.OneShape()


def holes(shape, z, d_min, d_max):
    """Centres of the closed loops of diameter d_min..d_max cut at height z."""
    sec = BRepAlgoAPI_Section(shape, gp_Pln(gp_Pnt(0, 0, z), gp_Dir(0, 0, 1)))
    sec.Build()
    pts = []
    e = TopExp_Explorer(sec.Shape(), TopAbs_EDGE)
    while e.More():
        c = BRepAdaptor_Curve(TopoDS.Edge(e.Current()))
        a, b = c.FirstParameter(), c.LastParameter()
        for i in range(21):
            q = c.Value(a + (b - a) * i / 20)
            pts.append((q.X(), q.Y()))
        e.Next()
    parent = list(range(len(pts)))

    def root(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for i in range(len(pts)):
        for j in range(i + 1, len(pts)):
            if abs(pts[i][0] - pts[j][0]) < 0.3 and abs(pts[i][1] - pts[j][1]) < 0.3:
                parent[root(i)] = root(j)
    groups = {}
    for i, p in enumerate(pts):
        groups.setdefault(root(i), []).append(p)
    out = []
    for g in groups.values():
        xs, ys = [p[0] for p in g], [p[1] for p in g]
        w, h = max(xs) - min(xs), max(ys) - min(ys)
        if d_min <= w <= d_max and abs(w - h) < 0.05:
            out.append(((max(xs) + min(xs)) / 2, (max(ys) + min(ys)) / 2, w))
    return sorted(out, key=lambda c: (round(c[1], 1), c[0]))


def bbox(shape):
    b = Bnd_Box()
    BRepBndLib.AddOptimal_s(shape, b, False, False)
    p0, p1 = b.CornerMin(), b.CornerMax()
    return tuple(round(v, 3) for v in (p0.X(), p0.Y(), p0.Z(), p1.X(), p1.Y(), p1.Z()))


def centre(cs):
    return sum(c[0] for c in cs) / len(cs), sum(c[1] for c in cs) / len(cs)


def main():
    folder = sys.argv[1] if len(sys.argv) > 1 else "."
    core = load(os.path.join(folder, CORE))
    board = load(os.path.join(folder, BOARD))

    # Baseboard: Ø2.1 holes; the inner four take the core's screws.
    bh = holes(board, 0.8, 2.0, 2.2)
    inner = [c for c in bh if abs(c[0]) < 14]
    mount = [c for c in bh if abs(c[0]) >= 14]
    # Core: its Ø1.7 screw holes, at mid-height.
    ch = holes(core, 73.0, 1.6, 1.8)
    assert len(inner) == 4 and len(mount) == 4 and len(ch) == 4, (bh, ch)

    # The core file has its closed face (with the screw counterbores) at
    # z = 67.85 and its open side, with the connectors that mate with the
    # baseboard, at z = 78.85. Turn it over about y (its hole pattern is
    # centred in x, so the footprint stays the same), then move its holes
    # onto the baseboard's with the open side down on z = 1.6.
    bx, by = centre(inner)
    cx, cy = centre(ch)
    rot = gp_Trsf()
    rot.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0)), math.pi)
    move = gp_Trsf()
    shift = (bx + cx, by - cy, 1.6 + 78.85)
    move.SetTranslation(gp_Vec(*shift))
    core_moved = BRepBuilderAPI_Transform(core, move.Multiplied(rot), True).Shape()

    comp = TopoDS_Compound()
    bld = BRep_Builder()
    bld.MakeCompound(comp)
    bld.Add(comp, board)
    bld.Add(comp, core_moved)

    w = STEPControl_Writer()
    w.Transfer(comp, STEPControl_AsIs)
    w.Write(os.path.join(folder, OUT + ".step"))
    BRepMesh_IncrementalMesh(comp, 0.02, False, 0.2, True)
    sw = StlAPI_Writer()
    sw.ASCIIMode = False
    sw.Write(comp, os.path.join(folder, OUT + ".stl"))

    mx, my = centre(mount)
    print("mounting holes Ø%.2f: %.2f (x) by %.2f (y), centre (%.3f, %.3f)" % (
        mount[0][2], max(c[0] for c in mount) - min(c[0] for c in mount),
        max(c[1] for c in mount) - min(c[1] for c in mount), mx, my))
    print("core screw holes: %.2f by %.2f; core turned over about y, then moved by (%.3f, %.3f, %.3f)" % (
        (max(c[0] for c in inner) - min(c[0] for c in inner),
         max(c[1] for c in inner) - min(c[1] for c in inner)) + shift))
    print("combined bounding box:", bbox(comp))
    print("wrote", OUT + ".step", OUT + ".stl")


if __name__ == "__main__":
    main()
