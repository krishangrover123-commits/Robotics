"""Shared RoboDK setup. Keep this file beside simple_task1/2/3.py.
Uses the supplied STL origins; STEP files cannot be substituted directly.
"""
import math
from pathlib import Path
from robodk.robolink import *
from robodk.robomath import *

CAD_DIR = Path.home() / "Downloads" / "CAD for Students"   # change if your STL files are elsewhere
APPROACH, CLEARANCE, BLEND = 50.0, 0.2, 1.0
RAPID_SPEED, JOINT_SPEED, TARGET_ROLL = 150, 60, 90
HOME_JOINTS = [0, -10, 10, 0, 90, 0]
TABLE_POSE = (800, 0, 100)
WORKTABLE_POSE = (203.1, 482.6, 100)

def pose_zx(p, z, hint):
    z = normalize3(z)
    x = normalize3(subs3(hint, mult3(z, dot(hint, z))))
    y = cross(z, x)
    return Mat([x + [0], y + [0], z + [0], list(p) + [1]]).tr()

def pose_zy(p, z, hint):
    return pose_zx(p, z, cross(hint, z))

def same_pose(a, b):
    return all(abs(a[i,j] - b[i,j]) < 1e-6 for i in range(3) for j in range(4))

def offset_z(pose, dz):
    return pose * transl(0, 0, dz)

def tool_geometry(cx, y0, cz):
    return transl(-cx, cz, -y0) * rotx(math.pi / 2)

class Steps(object):
    def __init__(self, prefix):
        self.prefix, self.k, self.items = prefix, 0, []

    def name(self):
        self.k += 1
        return "%s_%02d" % (self.prefix, self.k)

    def J(self, pose, name=None):
        self.items.append(("J", name or self.name(), pose))

    def L(self, pose, name=None):
        self.items.append(("L", name or self.name(), pose))

    def C(self, mid, end):
        self.items.append(("C", self.name(), mid, self.name(), end))

    def speed(self, v):
        self.items.append(("speed", v, JOINT_SPEED))

    def round(self, mm):
        self.items.append(("round", mm))


def table_bounds(table):
    """Read the actual library mesh, so an incorrect table cannot pass silently."""
    points = []
    pose = table.PoseAbs()
    for index in range(500):
        mesh, _ = table.GetPoints(FEATURE_OBJECT_MESH, index)
        if not mesh:
            break
        points.extend(pose * p[:3] for p in mesh)
    if not points:
        raise RuntimeError('Cannot measure the table. Check its mesh before building.')
    z = max(p[2] for p in points)
    top = [p for p in points if p[2] > z - 2]
    return min(p[0] for p in top), max(p[0] for p in top), min(p[1] for p in top), max(p[1] for p in top), z


def build(task, rdk=None):
    """Build one task without deleting items or overwriting station files."""
    rdk = rdk or Robolink()
    rdk.setRunMode(RUNMODE_SIMULATE)
    if rdk.Item('Worktable', ITEM_TYPE_FRAME).Valid():
        raise RuntimeError('Use a new station with one ABB IRB120 and one library table.')
    robots = rdk.ItemList(ITEM_TYPE_ROBOT)
    tables = [item for item in rdk.ItemList(ITEM_TYPE_OBJECT) if 'table' in item.Name().lower()]
    if len(robots) != 1 or 'IRB120' not in robots[0].Name().replace(' ', '').replace('-', '').upper():
        raise RuntimeError('Add exactly one ABB IRB120 robot.')
    if len(tables) != 1:
        raise RuntimeError('Add exactly one standard library table.')
    for spec in task['parts'] + [task['tool']]:
        if not (CAD_DIR / spec['file']).is_file():
            raise FileNotFoundError(CAD_DIR / spec['file'])
    robot, table = robots[0], tables[0]
    station = rdk.ActiveStation()
    base = robot.Parent()
    if base.Type() == ITEM_TYPE_FRAME:
        base.setParentStatic(station)
        base.setPose(eye(4))
    table.setParentStatic(station)
    table.setPose(transl(*TABLE_POSE))
    xmin, xmax, ymin, ymax, ztop = table_bounds(table)
    corner = (xmin, ymax, ztop)
    if max(abs(a-b) for a,b in zip(corner, WORKTABLE_POSE)) > 30:
        raise RuntimeError('This table differs from section 4.6. Use the 1.2m x 1m library table.')
    frame = rdk.AddFrame('Worktable', station)
    frame.setPose(transl(*WORKTABLE_POSE))
    to_frame = invH(frame.PoseAbs()) * robot.Parent().PoseAbs()
    for part in task['parts']:
        obj = rdk.AddFile(str(CAD_DIR / part['file']), frame)
        obj.setName(part['name'])
        obj.setPose(to_frame * part['pose'])
        sx, sz = part['size']
        for corner in ([0,0,0], [sx,0,0], [0,0,sz], [sx,0,sz]):
            p = obj.PoseAbs() * corner
            if not (xmin < p[0] < xmax and ymin < p[1] < ymax and p[2] >= ztop):
                raise RuntimeError(part['name'] + ' is not fully above the table.')
    spec = task['tool']
    tool = rdk.AddFile(str(CAD_DIR / spec['file']), robot)
    tool.setName(spec['name'])
    tool.setGeometryPose(spec['geom'])
    tool.setPoseTool(spec['tcp'])
    robot.setPoseTool(tool)
    robot.setPoseFrame(frame)
    robot.setJoints(HOME_JOINTS)
    home = rdk.AddTarget('Home', frame, robot)
    home.setAsJointTarget()
    home.setJoints(HOME_JOINTS)
    seed = list(HOME_JOINTS)

    def target(name, pose, joint=False):
        joints = robot.SolveIK(pose * invH(spec['tcp']), HOME_JOINTS if joint else seed).list()
        if len(joints) < 6:
            raise RuntimeError(name + ' is unreachable.')
        seed[:] = joints[:6]
        item = rdk.AddTarget(name, frame, robot)
        item.setAsCartesianTarget()
        item.setPose(to_frame * pose)
        item.setJoints(seed)
        return item

    programs = []
    for name, steps in task['programs']:
        program = rdk.AddProgram(name, robot)
        program.setPoseFrame(frame)
        program.setPoseTool(tool)
        for step in steps:
            kind = step[0]
            if kind == 'speed':
                program.setSpeed(step[1], step[2])
            elif kind == 'round':
                program.setRounding(step[1])
            elif kind == 'C':
                program.MoveC(target(step[1], step[2]), target(step[3], step[4]))
            else:
                point = target(step[1], step[2], kind == 'J')
                (program.MoveJ if kind == 'J' else program.MoveL)(point)
        programs.append(program)
    via = target(task['key'] + '_Via', task['via'], True) if 'via' in task else None
    main = rdk.AddProgram('MainProgram', robot)
    main.setPoseFrame(frame)
    main.setPoseTool(tool)
    main.setSpeed(RAPID_SPEED, JOINT_SPEED)
    main.MoveJ(home)
    for i, program in enumerate(programs):
        if i and via is not None:
            main.MoveJ(via)
        main.RunInstruction(program.Name(), INSTRUCTION_CALL_PROGRAM)
    main.MoveJ(home)
    rdk.setCollisionActive(COLLISION_ON)
    # The supplied mounting plate touches its own flange (link 6) by design.
    rdk.setCollisionActivePair(COLLISION_OFF, robot, tool, 6, 0)
    # Explicit pairs: never hide tool/workpiece or tool/table collisions.
    for obj in tables + [rdk.Item(p['name'], ITEM_TYPE_OBJECT) for p in task['parts']]:
        rdk.setCollisionActivePair(COLLISION_ON, tool, obj)
    results = {}
    # Validate the actual call sequence, including the inter-operation via moves.
    # Checking a subprogram from an arbitrary robot pose tests a different path.
    robot.setJoints(HOME_JOINTS)
    for program in [main]:
        result = program.Update(COLLISION_ON, timeout_sec=60, mm_step=1, deg_step=1)
        results[program.Name()] = result
        print(program.Name(), result, flush=True)
        if result[3] < 1:
            raise RuntimeError(program.Name() + ': ' + result[4])
    robot.setJoints(HOME_JOINTS)
    rdk.ShowMessage('Checks passed. Run MainProgram, then Save Station As ' + task['title'] + '_simple.rdk', False)
    return station, main, results


