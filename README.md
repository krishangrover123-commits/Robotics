# RoboDK robotics tasks

Python station builders for an ABB IRB120 performing welding, deburring and milling.
This is revision 2, before the later Task 3-only movement change.

## Files

- `simple_task1.py`: welding the two workpieces together.
- `simple_task2.py`: deburring the outer top edges and circular hole using the middle of the cylindrical tool.
- `simple_task3.py`: milling pockets A, B, C and side hole D.
- `simple_common.py`: shared station setup, geometry helpers and validation.
- `validation.json`: recorded revision 2 collision checks and simulation results.

Keep the four Python files in the same directory. The task scripts import the common file.

## Setup and use

1. Install RoboDK and its Python API (`python -m pip install -r requirements.txt` if needed).
2. Put the supplied STL models in `~/Downloads/CAD for Students`, or edit `CAD_DIR` in `simple_common.py`.
3. Create a new RoboDK station and add one ABB IRB 120-3/0.6 robot and the standard `Table-1200x1000mm-Welding.sld` table.
4. Run the required `simple_task1.py`, `simple_task2.py` or `simple_task3.py` using RoboDK's Python environment.
5. Run `MainProgram` and save the station under a new `.rdk` filename.

CAD models, robot/table library assets and saved stations are not included in this code repository.
The builders expect these STL files: `Task1_Part1.STL`, `Task1_Part2.STL`, `Task2_Part.STL`,
`Task3_Part.STL`, `AssemblyWeldingTorch.STL`, `AssemblySpindleSide.STL` and `AssemblyMillCutter.STL`.
Do not substitute STEP files without accounting for their different coordinate origins.

Saved `.rdk` stations contain the robot programmes and models and run independently of these Python builders.

## Motion settings

- Transfers: 50 mm/s; joint speed: 15 degrees/s.
- Linear acceleration: 100 mm/s²; joint acceleration: 30 degrees/s².
- Welding and milling feeds: 15 mm/s; deburring feed: 30 mm/s.
- Playback is set to 1× by the builder.
- Task 2 contacts the middle of the 40 mm abrasive cylinder at flange Z=143 mm, with a 30-degree tilt.

The subsequent Task 3-only 25 mm/s transfer and 8 degrees/s joint-speed revision is intentionally excluded.

## Validation and limitations

Revision 2 was tested in RoboDK 6.2.0.27007. All three complete MainProgram paths passed
collision checks at 1 mm / 1 degree sampling and completed simulation, returning Home.
The published source differs from that tested revision only in resolving the CAD folder
relative to the current user's home directory instead of a particular Windows username.

Tool/workpiece and tool/table collision checks are enabled. The tool against its own mounting
flange is excluded because the surfaces meet by design. The builder refuses to build over
an existing Worktable frame and does not overwrite saved stations.

A 0.2 mm simulation clearance is retained. This leaves a machining allowance and does not
prove exact finished dimensions or complete material removal. Collision sampling and simulation
do not guarantee assessment marks or suitability for operating physical equipment.

The assignment permits the workpieces to float above the table to represent unmodelled fixtures.
