################################################################################
# PyChrono – FEDA vehicle on a rigid terrain
#
# *  Irrlicht is used for real-time 3D visualization and user interaction.
# *  All vehicle components are visualized with their full triangle meshes.
# *  A rigid terrain patch is created and provided with a custom texture.
# *  A keyboard driver allows interactive steering/throttle/braking.
# *  The rendering loop runs at 50 FPS while the dynamics are integrated with a
#    smaller fixed step for numerical stability.
################################################################################
#
# Tested with:  Chrono 7.0+  (Python bindings)
#
################################################################################
import math
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

# ------------------------------------------------------------------------------
# 1. Global simulation parameters
# ------------------------------------------------------------------------------
contact_method          = chrono.ChContactMethod_NSC   # or _SMC
vehicle_vis_type        = veh.VisualizationType_MESH   # mesh for all parts
tire_vis_type           = veh.VisualizationType_MESH
powertrain_model        = veh.PowertrainModelType_SHAFTS
tire_model              = veh.TireModelType.TMEASY

# Initial vehicle location & orientation
init_loc   = chrono.ChVectorD(0, 0, 0.5)               # x,y,z [m]
init_yaw   = 0.0                                       # [rad]

# Time settings
step_size          = 1.0e-3          # integration step (dynamics)
render_fps         = 50              # render frequency [frames/s]
render_interval    = 1.0 / render_fps
simulation_end     = 60.0            # [s] – press ESC any time to quit

# Irrlicht chase-camera parameters
chase_dist   = 6.0      # distance to target point [m]
chase_height = 1.2      # camera height above target [m]

# Path to Chrono data (textures, meshes, …)
chrono.SetChronoDataPath(chrono.GetChronoDataPath())

# ------------------------------------------------------------------------------
# 2. Create the physical system
# ------------------------------------------------------------------------------
system = chrono.ChSystemNSC() if contact_method == chrono.ChContactMethod_NSC \
         else chrono.ChSystemSMC()

# ------------------------------------------------------------------------------
# 3. Rigid terrain with custom visual material
# ------------------------------------------------------------------------------
terrain = veh.RigidTerrain(system)

ground_mat = chrono.ChMaterialSurfaceNSC()  # friction, restitution can be set
patch = terrain.AddPatch(
    ground_mat,
    chrono.ChVectorD(0, 0, 0),             # location
    chrono.ChVectorD(0, 0, 1),             # normal
    300, 300)                              # size X,Y [m]

# Visual texture
patch.GetVisualShape().SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"), 200, 200)
terrain.Initialize()

# ------------------------------------------------------------------------------
# 4. Create the FEDA vehicle
# ------------------------------------------------------------------------------
vehicle = veh.FEDA(system,
                   contact_method,
                   powertrain_model,
                   tire_model,
                   veh.ChassisCollisionType_NONE)

vehicle.SetChassisVisualizationType(vehicle_vis_type)
vehicle.SetSuspensionVisualizationType(vehicle_vis_type)
vehicle.SetSteeringVisualizationType(vehicle_vis_type)
vehicle.SetWheelVisualizationType(vehicle_vis_type)
vehicle.SetTireVisualizationType(tire_vis_type)

vehicle.Initialize(chrono.ChCoordsysD(init_loc,
                                      chrono.ChQuaternionD(chrono.Q_from_AngZ(init_yaw))))

# ------------------------------------------------------------------------------
# 5. Irrlicht application & GUI driver
# ------------------------------------------------------------------------------
app = veh.ChWheeledVehicleIrrApp(vehicle.GetVehicle(),
                                 "FEDA on Rigid Terrain",
                                 irr.dimension2du(1280, 720))

# Attach the terrain to Irrlicht so it also gets rendered
app.SetTerrain(terrain)

app.AddTypicalLights()
app.SetChaseCamera(vehicle.GetChassis().GetFrame_REF_to_abs().GetPos(),
                   chase_dist, chase_height)
app.SetSkyBox()                      # simple sky dome
app.AddLogo()                        # Chrono logo on the screen

# Driver: W-S-A-D (throttle, brake, left, right) + space for hand-brake
driver = veh.ChIrrGuiDriver(app)
driver.SetSteeringDelta(2.5 * chrono.CH_C_DEG_TO_RAD)   # radians per key press
driver.SetThrottleDelta(0.05)
driver.SetBrakingDelta(0.2)
driver.Initialize()

# ------------------------------------------------------------------------------
# 6. Simulation loop
# ------------------------------------------------------------------------------
print("======================================================")
print("Controls: W/S throttle | A/D steering | SPACE brake")
print("Esc to quit.")
print("======================================================")

# Rendering control variables
time = 0.0
render_step = max(1, math.floor(1.0 / (render_fps * step_size)))
step_number = 0

while app.GetDevice().run():
    time = system.GetChTime()
    if time >= simulation_end:
        break

    # Render at the desired FPS
    if step_number % render_step == 0:
        app.BeginScene(True, True, irr.SColor(255, 140, 161, 192))
        app.DrawAll()
        app.EndScene()

    # Collect driver inputs
    driver_inputs = driver.GetInputs()

    # Synchronize modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    app.Synchronize("", driver_inputs)

    # Advance state
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    system.DoStepDynamics(step_size)

    step_number += 1

# ------------------------------------------------------------------------------
# 7. Clean up
# ------------------------------------------------------------------------------
print("Simulation ended at t = {:.2f} s".format(time))