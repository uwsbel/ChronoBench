Below is a cleaned-up and fully functional PyChrono script that incorporates the requested changes:

• fixes several minor issues in the original file (un-used driver object, missing driver input construction, redundant calls, etc.);  
• moves the UAZ-BUS starting point to (-40,0,0.5);  
• replaces the terrain texture with “concrete.jpg”;  
• adds an autonomous double-lane-change manoeuvre that first accelerates, then performs two opposite steering actions, and finally brakes to a stop.

```
import math
import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh

# ---------------------------------------------------------------------------
# Misc. Chrono paths
# ---------------------------------------------------------------------------
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# ---------------------------------------------------------------------------
# Vehicle parameters
# ---------------------------------------------------------------------------
initLoc = chrono.ChVector3d(-40, 0, 0.5)          # << moved 40 m back
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

vis_type               = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE
tire_model             = veh.TireModelType_TMEASY

# ---------------------------------------------------------------------------
# Terrain parameters
# ---------------------------------------------------------------------------
terrainHeight = 0.0
terrainLength = 100.0
terrainWidth  = 100.0

# ---------------------------------------------------------------------------
# Simulation control
# ---------------------------------------------------------------------------
contact_method   = chrono.ChContactMethod_NSC
step_size        = 1e-3
tire_step_size   = step_size
render_step_size = 1.0 / 50.0                         # 50 FPS

trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)

# ---------------------------------------------------------------------------
# Create the vehicle
# ---------------------------------------------------------------------------
vehicle = veh.UAZBUS()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# ---------------------------------------------------------------------------
# Create the rigid-terrain patch
# ---------------------------------------------------------------------------
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrainHeight), chrono.QUNIT),
    terrainLength,
    terrainWidth,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# ---------------------------------------------------------------------------
# Visualization system (Irrlicht)
# ---------------------------------------------------------------------------
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("UAZBUS – Double Lane Change Demo")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# ---------------------------------------------------------------------------
# Double lane-change manoeuvre generator
# ---------------------------------------------------------------------------
def GetManeuverInputs(time):
    """
    Return steering, throttle and braking values that realise a double lane
    change followed by braking to a stop.
    """
    steering = 0.0
    throttle = 0.0
    braking  = 0.0

    # 0 – 2 s : accelerate straight
    if time < 2.0:
        throttle = 0.7

    # 2 – 3.5 s : steer left
    elif time < 3.5:
        throttle = 0.7
        steering = 0.5

    # 3.5 – 5 s : straighten
    elif time < 5.0:
        throttle = 0.7

    # 5 – 6.5 s : steer right
    elif time < 6.5:
        throttle = 0.7
        steering = -0.5

    # 6.5 – 8 s : straighten again
    elif time < 8.0:
        throttle = 0.7

    # 8 – 10 s : braking phase
    elif time < 10.0:
        braking = 0.8

    # 10+ s : stay stopped
    return steering, throttle, braking

# ---------------------------------------------------------------------------
# Simulation loop
# ---------------------------------------------------------------------------
print("VEHICLE MASS:", vehicle.GetVehicle().GetMass())

driver_inputs = veh.ChDriverInputs()               # allocate once
render_steps  = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number    = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render scene
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    # Driver logic --------------------------------------------------------
    s, t, b = GetManeuverInputs(time)
    driver_inputs.m_steering = s
    driver_inputs.m_throttle = t
    driver_inputs.m_braking  = b

    # Synchronize modules -------------------------------------------------
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance the simulation ---------------------------------------------
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Real-time pacing ----------------------------------------------------
    realtime_timer.Spin(step_size)
    step_number += 1
```