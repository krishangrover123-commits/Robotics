"""Task 2: Deburring. Run in a new station containing one ABB IRB120 and the library table."""
from simple_common import *

T2_POS = (225, 100, 225)
DEBURR_TILT, DEBURR_SPEED, T2_HOLE_DIR = 45, 30, -1

T2_PART = transl(*T2_POS) * rotx(math.radians(90))
T2_TOOL = dict(file='AssemblySpindleSide.STL', name='SpindleSide', geom=tool_geometry(40.0, 31.99, 71.64), tcp=transl(-20.0, 0, 163.0) * roty(math.radians(-DEBURR_TILT)) * rotz(math.radians(TARGET_ROLL)))

def deburr_pose(p, m, lean_in=False):
    """Edge point p, m = horizontal normal of the side wall pointing into free air."""
    up = [0, 1, 0]
    q = add3(p, mult3(normalize3(add3(up, m)), CLEARANCE))
    return T2_PART * pose_zx(q, [0, -1, 0], mult3(m, -1) if lean_in else m) * rotz(math.radians(TARGET_ROLL))

def plan_task2():
    top = 50.0
    corners = [[0, top, 200], [0, top, 0], [200, top, 0], [200, top, 200]]
    normals = [[-1, 0, 0], [0, 0, -1], [1, 0, 0], [0, 0, 1]]
    s = Steps('T2_Outer')
    s.speed(RAPID_SPEED)
    first = deburr_pose(corners[0], normals[0])
    s.J(offset_z(first, -APPROACH), 'T2_Outer_Approach')
    s.L(first)
    s.speed(DEBURR_SPEED)
    for i in range(4):
        a, b, m = (corners[i], corners[(i + 1) % 4], normals[i])
        if i > 0:
            s.L(deburr_pose(a, m))
        last = deburr_pose(b, m)
        s.L(last)
    s.speed(RAPID_SPEED)
    s.L(offset_z(last, -APPROACH), 'T2_Outer_Retract')
    outer = s.items
    cx, cz, r = (100.0, 100.0, 60.0)

    def rim(a):
        rad = [math.cos(math.radians(a)), 0, math.sin(math.radians(a))]
        return deburr_pose([cx + r * rad[0], top, cz + r * rad[2]], mult3(rad, -1), True)
    centre = T2_PART * transl(cx, top, cz)
    start = math.degrees(math.atan2(centre[1, 3], -centre[0, 3]))
    s = Steps('T2_Hole')
    s.speed(RAPID_SPEED)
    s.J(offset_z(rim(start), -APPROACH), 'T2_Hole_Approach')
    s.L(rim(start))
    s.speed(DEBURR_SPEED)
    for q in range(4):
        a0 = start + T2_HOLE_DIR * 90.0 * q
        s.round(BLEND if q < 3 else -1)
        s.C(rim(a0 + T2_HOLE_DIR * 45), rim(a0 + T2_HOLE_DIR * 90))
    s.speed(RAPID_SPEED)
    s.L(offset_z(rim(start), -APPROACH), 'T2_Hole_Retract')
    hole = s.items
    parts = [dict(file='Task2_Part.STL', name='Task2_Part', pose=T2_PART, size=(200, 200))]
    return dict(key='T2', title='Task2_Deburring', tool=T2_TOOL, parts=parts, programs=[('Deburr_Outer_T2', outer), ('Deburr_Hole_T2', hole)])

if __name__ == "__main__":
    build(plan_task2())
