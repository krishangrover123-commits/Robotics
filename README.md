# RoboDK Project

Python station builders for an ABB IRB120 doing welding, deburring and milling. Each task builds as its own RoboDK station.

## Files

- `simple_task1.py`: welds the two workpieces together (top profile, back, left and front seams).
- `simple_task2.py`: deburrs the outer top edges and the circular hole edge with the spindle tilted 45°.
- `simple_task3.py`: mills pockets A, B and C and side hole D.
- `simple_common.py`: shared geometry helpers, station setup and validation.

## Setup and use

1. Install RoboDK and its Python API (`python -m pip install -r requirements.txt` if needed).
2. Put the supplied STL models in `~/Downloads/CAD for Students`, or edit `CAD_DIR` in `simple_common.py`.
3. Create a new RoboDK station. Add one ABB IRB 120-3/0.6 robot and the standard `Table-1200x1000mm-Welding` library table. Other table models are rejected, because the section 4.6 coordinates assume this one.
4. Run `simple_task1.py`, `simple_task2.py` or `simple_task3.py` in RoboDK's Python environment.
5. Run `MainProgram`, then save the station under a new `.rdk` filename.

CAD models, robot and table library files, and saved stations are not included in this repository. The builders expect these STL files: `Task1_Part1.STL`, `Task1_Part2.STL`, `Task2_Part.STL`, `Task3_Part.STL`, `AssemblyWeldingTorch.STL`, `AssemblySpindleSide.STL` and `AssemblyMillCutter.STL`. Don't substitute the STEP files: their coordinate origins differ.

## Configuration

- **Table and Worktable frame:** table at (800, 0, 100); Worktable frame on its corner at (203.1, 482.6, 100), as in section 4.6. Workpieces float above the table, which the assignment allows to represent unmodelled fixtures.
- **Task 1:** TCP 10 mm in front of the bent torch nozzle. The torch is perpendicular to each seam with a 15° push angle. Between faces it moves to a via point 175 mm above the part.
- **Task 2:** the TCP is the corner of the Ø40 cylinder's end face (163 mm from the flange), with the tool tilted 45°. On the outer edge the tool leans over the part; on the hole edge it leans in over the hole.
- **Task 3:** Ø8 cutter with the TCP at its tip. Pocket B is cut in two 10 mm layers.
- **Speeds:** transfers 150 mm/s and 60°/s; welding and milling feed 15 mm/s; deburring feed 30 mm/s.
- **Smooth arcs:** tangent arc joins are blended with 1 mm rounding. Sharp corners keep exact stops.

## Validation and limitations

Each builder checks that every target is reachable. It then runs `MainProgram`, including the transfer moves, with collision checking at 1 mm / 1° sampling, and stops with an error if anything fails.

Collision checking covers the tool against the workpiece and against the table. Only the tool against its own mounting flange (robot link 6) is excluded, because those surfaces meet by design. The builder refuses to build over an existing Worktable frame and never overwrites saved stations.

A 0.2 mm clearance is kept on deburred edges and milled walls and floors, so mesh contact isn't flagged as a collision. Because of that, the pockets are cut to 9.8 / 19.8 mm, not the nominal 10 / 20 mm. Simulation checks the sampled paths and geometry. It does not prove exact finished dimensions or material removal, guarantee assessment marks, or show the programs are safe to run on a physical robot.
