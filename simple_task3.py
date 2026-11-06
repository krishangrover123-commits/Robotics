"""Task 3: Milling. Run in a new station containing one ABB IRB120 and the library table."""
from simple_common import *

T3_POS = (333.1, -15, 130)
MILL_SPEED, MAX_STEP_DOWN = 15, 10

T3_PART = transl(*T3_POS) * rotz(math.radians(-90)) * rotx(math.radians(90))
T3_TOOL = dict(file='AssemblyMillCutter.STL', name='MillCutter', geom=tool_geometry(40.0, 32.71, 66.63), tcp=transl(0, 0, 120.0) * rotz(math.radians(TARGET_ROLL)))
R_CUT = 4.0
T3_X_HINT = [0, 0, 1]

def mill_down(x, y, z):
    return T3_PART * pose_zx([x, y, z], [0, -1, 0], T3_X_HINT) * rotz(math.radians(TARGET_ROLL))

def layers(top, floor):
    depth = top - floor
    n = max(1, int(math.ceil(depth / MAX_STEP_DOWN - 1e-09)))
    return [top - depth * (i + 1) / n for i in range(n)]

def spread(a, b, max_step):
    n = max(1, int(math.ceil(abs(b - a) / max_step - 1e-09)))
    return [a + (b - a) * i / n for i in range(n + 1)]

def plan_pocket_a():
    s = Steps('T3_A')
    x0, x1 = (10 + R_CUT + CLEARANCE, 60 - R_CUT - CLEARANCE)
    z0, z1 = (60 + R_CUT + CLEARANCE, 90 - R_CUT - CLEARANCE)
    s.speed(RAPID_SPEED)
    s.J(mill_down(x0, 50 + APPROACH, z0), 'T3_A_Approach')
    for y in layers(50.0, 40.0 + CLEARANCE):
        s.speed(MILL_SPEED)
        s.L(mill_down(x0, y, z0))
        zs = spread(z0, z1, 6.0)
        for i, z in enumerate(zs):
            xa, xb = (x0, x1) if i % 2 == 0 else (x1, x0)
            if i > 0:
                s.L(mill_down(xa, y, z))
            s.L(mill_down(xb, y, z))
        xe = x1 if len(zs) % 2 == 1 else x0
        for x, z in [(xe, z0), (x0 + x1 - xe, z0), (x0 + x1 - xe, z1), (xe, z1)]:
            s.L(mill_down(x, y, z))
        s.speed(RAPID_SPEED)
        s.L(mill_down(xe, 50 + 5, z1))
    s.L(mill_down(xe, 50 + APPROACH, z1), 'T3_A_Retract')
    return ('Mill_PocketA_T3', s.items)

def plan_pocket_b():
    s = Steps('T3_B')
    c1, c2, zc = (25.0, 40.0, 25.0)
    r_out = 15.0 - R_CUT - CLEARANCE
    rings = [r_out / 2.0, r_out]
    s.speed(RAPID_SPEED)
    s.J(mill_down(c1, 50 + APPROACH, zc), 'T3_B_Approach')
    for y in layers(50.0, 30.0 + CLEARANCE):
        s.speed(MILL_SPEED)
        s.L(mill_down(c1, y, zc))
        s.L(mill_down(c2, y, zc))
        for r in rings:
            s.L(mill_down(c2, y, zc - r))
            s.round(BLEND)
            s.C(mill_down(c2 + r, y, zc), mill_down(c2, y, zc + r))
            s.L(mill_down(c1, y, zc + r))
            s.C(mill_down(c1 - r, y, zc), mill_down(c1, y, zc - r))
            s.round(-1)
            s.L(mill_down(c2, y, zc - r))
        s.L(mill_down(c2, y, zc))
        s.L(mill_down(c1, y, zc))
    s.speed(RAPID_SPEED)
    s.L(mill_down(c1, 50 + APPROACH, zc), 'T3_B_Retract')
    return ('Mill_PocketB_T3', s.items)

def plan_pocket_c():
    s = Steps('T3_C')
    x_in, x_out = (70 + R_CUT + CLEARANCE, 100 + R_CUT + 3.0)
    z0, z1 = (10 + R_CUT + CLEARANCE, 40 - R_CUT - CLEARANCE)
    s.speed(RAPID_SPEED)
    s.J(mill_down(x_out, 50 + APPROACH, z0), 'T3_C_Approach')
    for y in layers(50.0, 40.0 + CLEARANCE):
        s.L(mill_down(x_out, y, z0))
        s.speed(MILL_SPEED)
        zs = spread(z0, z1, 6.0)
        for i, z in enumerate(zs):
            xa, xb = (x_out, x_in) if i % 2 == 0 else (x_in, x_out)
            if i > 0:
                s.L(mill_down(xa, y, z))
            s.L(mill_down(xb, y, z))
        if len(zs) % 2 == 0:
            s.L(mill_down(x_in, y, z1))
        s.L(mill_down(x_in, y, z0))
        s.L(mill_down(x_out, y, z0))
        s.speed(RAPID_SPEED)
    s.L(mill_down(x_out, 50 + APPROACH, z0), 'T3_C_Retract')
    return ('Mill_PocketC_T3', s.items)

def plan_hole_d():
    s = Steps('T3_D')
    yc, zc, r = (25.0, 70.0, 10.0 - R_CUT - CLEARANCE)
    into = [-1, 0, 0]
    hint = [0, 0, -1]

    def side(x, y, z):
        return T3_PART * pose_zx([x, y, z], into, hint) * rotz(math.radians(TARGET_ROLL))
    s.speed(RAPID_SPEED)
    s.J(side(100 + 80, yc, zc), 'T3_D_Approach')
    s.L(side(100 + 3, yc, zc))
    s.speed(MILL_SPEED)
    for x in layers(100.0, 90.0 + CLEARANCE):
        s.L(side(x, yc, zc))
        s.L(side(x, yc, zc + r))
        s.round(BLEND)
        s.C(side(x, yc + r, zc), side(x, yc, zc - r))
        s.round(-1)
        s.C(side(x, yc - r, zc), side(x, yc, zc + r))
        s.L(side(x, yc, zc))
    s.speed(RAPID_SPEED)
    s.L(side(100 + 80, yc, zc), 'T3_D_Retract')
    return ('Mill_HoleD_T3', s.items)

def plan_task3():
    via = T3_PART * pose_zx([150, 120, 70], normalize3([-1, -1, 0]), T3_X_HINT) * rotz(math.radians(TARGET_ROLL))
    hole = plan_hole_d()
    hole[1].insert(1, ('J', 'T3_D_Via', via))
    parts = [dict(file='Task3_Part.STL', name='Task3_Part', pose=T3_PART, size=(100, 100))]
    return dict(key='T3', title='Task3_Milling', tool=T3_TOOL, parts=parts, programs=[plan_pocket_a(), plan_pocket_b(), plan_pocket_c(), hole])

if __name__ == "__main__":
    build(plan_task3())
