"""Task 1: Welding. Run in a new station containing one ABB IRB120 and the library table."""
from simple_common import *

T1_POS = (403.1, 100, 190)
PUSH_ANGLE, WELD_SPEED = 15, 15
T1_HINT_TOP = [-0.866, 0, 0.5]
T1_HINT_BACK = T1_HINT_FRONT = [-0.707, 0.707, 0]
T1_HINT_LEFT = [0, 0.5, -0.866]
T1_VIA_HEIGHT = 175

T1_PART = transl(*T1_POS) * rotx(math.radians(90))
T1_TOOL = dict(file='AssemblyWeldingTorch.STL', name='WeldingTorch', geom=tool_geometry(40.0, 26.94, 70.46), tcp=transl(0, 40.0, 159.28) * rotx(math.radians(-30)))

def weld_pose(p, n, travel, y_hint):
    """Torch perpendicular to the surface (normal n), tilted PUSH_ANGLE towards travel."""
    a = math.radians(PUSH_ANGLE)
    z = add3(mult3(n, -math.cos(a)), mult3(normalize3(travel), math.sin(a)))
    return pose_zy(p, z, y_hint)

def weld_program(prefix, segs, arc_centre=None):
    """segs: (kind, points, surface normal, torch y-axis hint). 'L' = line p0->p1,
    'C' = arc p0 -> p1 (mid) -> p2 around arc_centre (angle decreasing)."""

    def tangent(kind, pts, i):
        if kind == 'L':
            return subs3(pts[1], pts[0])
        return cross([0, 1, 0], subs3(pts[i], arc_centre))

    def lifted(pts_, n, hint, travel, i):
        """Approach / retract point: off the surface and above the seam."""
        p = add3(add3(pts_[i], mult3(n, 20.0)), [0, APPROACH, 0])
        return T1_PART * weld_pose(p, n, travel, hint)
    s = Steps(prefix)
    s.speed(RAPID_SPEED)
    last = None
    kinds = [sg[0] for sg in segs]
    for i, (kind, pts, n, hint) in enumerate(segs):
        start = T1_PART * weld_pose(pts[0], n, tangent(kind, pts, 0), hint)
        if last is None:
            s.J(lifted(pts, n, hint, tangent(kind, pts, 0), 0), prefix + '_Approach')
            s.L(start)
            s.speed(WELD_SPEED)
        elif not same_pose(start, last):
            s.L(start)
        if kind == 'L':
            last = T1_PART * weld_pose(pts[1], n, tangent(kind, pts, 1), hint)
            s.L(last)
        else:
            if i + 1 < len(segs) and kinds[i + 1] == 'C' and (i == 0 or kinds[i - 1] != 'C'):
                s.round(BLEND)
            elif i > 0 and kinds[i - 1] == 'C' and (i + 1 == len(segs) or kinds[i + 1] != 'C'):
                s.round(-1)
            mid = T1_PART * weld_pose(pts[1], n, tangent(kind, pts, 1), hint)
            last = T1_PART * weld_pose(pts[2], n, tangent(kind, pts, 2), hint)
            s.C(mid, last)
    s.speed(RAPID_SPEED)
    s.L(lifted(pts, n, hint, tangent(kind, pts, len(pts) - 1), len(pts) - 1), prefix + '_Retract')
    return s.items

def plan_task1():
    nt, nb, nl, nf = ([0, 1, 0], [0, 0, -1], [-1, 0, 0], [0, 0, 1])
    r, cx, cz = (35.0, 100.0, 150.0)
    top = [('L', [[100, 40, 200], [100, 40, 185]], nt, T1_HINT_TOP)]
    for a0, a1 in ((90, 0), (0, -90)):
        pts = [[cx + r * math.cos(math.radians(a)), 40, cz + r * math.sin(math.radians(a))] for a in (a0, (a0 + a1) / 2.0, a1)]
        top.append(('C', pts, nt, T1_HINT_TOP))
    top += [('L', [[100, 40, 115], [100, 40, 100]], nt, T1_HINT_TOP), ('L', [[100, 40, 100], [160, 40, 65]], nt, T1_HINT_TOP), ('L', [[160, 40, 65], [100, 40, 30]], nt, T1_HINT_TOP), ('L', [[100, 40, 30], [100, 40, 0]], nt, T1_HINT_TOP)]
    back = [('L', [[0, 20, 0], [100, 20, 0]], nb, T1_HINT_BACK), ('L', [[100, 20, 0], [100, 40, 0]], nb, T1_HINT_BACK)]
    left = [('L', [[0, 20, 0], [0, 20, 200]], nl, T1_HINT_LEFT)]
    front = [('L', [[0, 20, 200], [100, 20, 200]], nf, T1_HINT_FRONT), ('L', [[100, 20, 200], [100, 40, 200]], nf, T1_HINT_FRONT)]
    parts = [dict(file='Task1_Part1.STL', name='Task1_Part1', pose=T1_PART, size=(200, 200)), dict(file='Task1_Part2.STL', name='Task1_Part2', size=(160, 200), pose=T1_PART * transl(0, 20, 0))]
    via = T1_PART * pose_zy([100, 40 + T1_VIA_HEIGHT, 100], [0, -1, 0], T1_HINT_TOP)
    return dict(key='T1', title='Task1_Welding', tool=T1_TOOL, parts=parts, via=via, programs=[('Weld_Top_T1', weld_program('T1_Top', top, [cx, 40, cz])), ('Weld_Back_T1', weld_program('T1_Back', back)), ('Weld_Left_T1', weld_program('T1_Left', left)), ('Weld_Front_T1', weld_program('T1_Front', front))])

if __name__ == "__main__":
    build(plan_task1())
