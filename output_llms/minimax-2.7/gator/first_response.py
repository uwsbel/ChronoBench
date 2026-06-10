#!/usr/bin/env python3
"""
Gator vehicle simulation on a flat rigid terrain using PyChrono + Irrlicht.

Features
--------
* Gator vehicle initialised with a TMeasy tyre model.
* Rigid terrain (200 × 200 m) with a built‑in Chrono texture.
* Mesh visualisation for chassis, wheels and tyres.
* Interactive driver (arrow‑keys) for steering, throttle and braking.
* Real‑time loop at 50 fps (20 ms time‑step).

Prerequisites
-------------
* PyChrono (chronoengine/chrono or pip install pychrono).
* Chrono data files (textures, vehicle meshes) – the script points to the
  default Chrono data folder.  Adjust ``chrono.SetDataPath`` if your
  installation uses a different location.

How to run
----------
``python run_gator.py``  – a window will appear; use the arrow keys to
drive the Gator.
"""

# ----------------------------------------------------------------------
# 1.  Imports
# ----------------------------------------------------------------------
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

# ----------------------------------------------------------------------
# 2.  Path to Chrono data (textures, meshes, …)
# ----------------------------------------------------------------------
# Adjust the string to the folder that contains the Chrono data set.
chrono.SetDataPath("C:/Chrono/data/")   # <-- adapt to your installation

# ----------------------------------------------------------------------
# 3.  Create a physical system (non‑smooth contact, NSC)
# ----------------------------------------------------------------------
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVector(0, -9.81, 0))   # gravity

# ----------------------------------------------------------------------
# 4.  Rigid terrain – flat plane with a texture
# ----------------------------------------------------------------------
terrain = veh.RigidTerrain(system)

# Add a rectangular patch (200 m × 200 m)
patch = terrain.AddPatch(
    chrono.ChVector(0, 0, 0),                # centre of the patch
    chrono.ChQuaternion(1, 0, 0, 0),         # no rotation
    200,                                      # size in X
    200                                       # size in Y
)

# Mechanical properties of the ground
patch.SetFriction(0.9)
patch.SetRestitution(0.0)

# Use a built‑in Chrono texture (tile4.png) – replace with a custom file if needed
patch.SetTexture(chrono.GetChronoDataFile("textures/tile4.png"))

terrain.Initialize()

# ----------------------------------------------------------------------
# 5.  Gator vehicle – location, orientation, contact method, tyre model
# ----------------------------------------------------------------------
# Initial pose of the vehicle
init_loc = chrono.ChVector(0.0, 0.5, 0.0)    # slightly above ground
init_rot = chrono.ChQuaternion(1, 0, 0, 0)    # no rotation

# Instantiate the Gator using a TMeasy tyre model
gator = veh.Gator(init_loc, init_rot, veh.TMeasyTire)

# Choose mesh visualisation for every component
gator.SetChassisVisualizationType(veh.ChassisVisType.Mesh)
gator.SetWheelVisualizationType(veh.WheelVisType.Mesh)
gator.SetTireVisualizationType(veh.TireVisType.Mesh)

# Initialise the vehicle in the system
gator.Initialize(system)

# ----------------------------------------------------------------------
# 6.  Interactive driver (keyboard → steering, throttle, brake)
# ----------------------------------------------------------------------
# The Irrlicht‑based interactive driver reads arrow‑key input.
driver = veh.ChIrrVehicleInteractiveDriver(gator.GetVehicle())
driver.Initialize()

# ----------------------------------------------------------------------
# 7.  Irrlicht application (window, lights, camera, sky‑box)
# ----------------------------------------------------------------------
app = irr.ChIrrApp(
    system,
    "Gator – PyChrono Demo",               # window title
    irr.dimension2du(1280, 720)            # window resolution
)

# Add a sky‑box, standard lights and a camera that looks at the vehicle
app.SetSkyBox()
app.AddTypicalLights()
app.AddCamera(irr.vector3df(0, 5, -8), irr.vector3df(0, 1, 0))

# Attach the driver so that it receives keyboard events
app.SetDriver(driver)

# ----------------------------------------------------------------------
# 8.  Simulation timing – 50 Hz real‑time
# ----------------------------------------------------------------------
app.SetTimestep(0.02)      # 20 ms → 50 fps
app.SetRealTime(True)      # keep the simulation in sync with wall‑clock time

# ----------------------------------------------------------------------
# 9.  Main simulation loop
# ----------------------------------------------------------------------
# We can either use app.Start() (blocks) or run a custom loop.
# Below is a custom loop that shows how to step manually while preserving
# real‑time behaviour.

device = app.GetDevice()
while device.run():
    app.BeginScene()
    app.DrawAll()
    app.EndScene()

    # Synchronise with real time and advance the dynamics by one time‑step
    app.Synchronize()
    app.Advance(app.GetTimestep())

# When the user closes the Irrlicht window the program ends.
print("Simulation finished.")