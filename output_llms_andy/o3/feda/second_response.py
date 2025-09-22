#
#   FEDA – ISO Double Lane-Change Test
#
#   This script corrects a few minor issues in the original file and implements the
#   requested modifications:
#       1.  Replaces the interactive‐keyboard driver with a combined
#           path-follower / cruise-control driver.
#       2.  Builds an ISO-like double lane-change path (using a Bézier curve).
#       3.  Moves the vehicle start position to (-50 , 0 , 0.5).
#       4.  Extends the terrain length to 200 m.
#
#   Tested with PyChrono 8.0+.
#

import math
import pychrono.core   as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh

# ---------------------------------------------------------------------
# Global Chrono initialisation
# ---------------------------------------------------------------------
chrono.SetChronoDataPath(chrono.GetChronoDataPath())          # make sure we know where the Chrono data folder is
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')      # and where the vehicle data live

# ---------------------------------------------------------------------
# Simulation parameters
# ---------------------------------------------------------------------
step_size        = 1e-3                      # integrator step
render_fps       = 50                        # Irrlicht render FPS
render_step_size = 1.0 / render_fps
tire_step_size   = step_size

# ---------------------------------------------------------------------
# Vehicle initial position (changed according to the request)
# ---------------------------------------------------------------------
initLoc = chrono.ChVector3d(-50, 0.0, 0.5)   # x, y, z
initRot = chrono.ChQuaterniond(1, 0, 0, 0)   # no initial yaw

# ---------------------------------------------------------------------
# Terrain (length increased according to the request)
# ---------------------------------------------------------------------
terrainHeight = 0.0
terrainLength = 200.0        # changed (was 100)
terrainWidth  = 100.0

# ---------------------------------------------------------------------
# Create the FEDA vehicle
# ---------------------------------------------------------------------
vis_type               = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE
tire_model             = veh.TireModelType_TMEASY
contact_method         = chrono.ChContactMethod_NSC

vehicle = veh.FEDA()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()

# set visualisation types
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# (Optional) use Bullet collision system
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# ---------------------------------------------------------------------
# Terrain patch
# ---------------------------------------------------------------------
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
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 400, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# ---------------------------------------------------------------------
# Create ISO DOUBLE-LANE-CHANGE path ----------------------------------
# ---------------------------------------------------------------------
#
# A *very* simplified geometric representation:
#   * Start straight.
#   * Shift +3.5 m (left lane change).
#   * Shift –3.5 m (right lane change).
#   * Return to centre lane.
#

path_pts = [
    chrono.ChVector3d(-50,  0.0, 0.0),
    chrono.ChVector3d(-40,  0.0, 0.0),
    chrono.ChVector3d(-30,  0.0, 0.0),

    chrono.ChVector3d(-20,  3.5, 0.0),     # first lane change
    chrono.ChVector3d(-10,  3.5, 0.0),

    chrono.ChVector3d(   0, -3.5, 0.0),    # second lane change
    chrono.ChVector3d(  10, -3.5, 0.0),

    chrono.ChVector3d(  20,  0.0, 0.0),    # back to centre
    chrono.ChVector3d(  30,  0.0, 0.0),
    chrono.ChVector3d(  50,  0.0, 0.0),
]

bezier_path = chrono.ChBezierCurve()
for p in path_pts:
    bezier_path.AddPoint(p)
bezier_path.SetClosed(False)   # open curve

# ---------------------------------------------------------------------
# Path-Follower / Cruise-Control driver
# ---------------------------------------------------------------------
target_speed = 10.0    # m/s

driver = veh.ChPathFollowerDriver(vehicle.GetVehicle(),
                                  bezier_path,
                                  "ISO_double_lane_change",
                                  target_speed)

# Steering controller setup
driver.GetSteeringController().SetLookAheadDistance(5.0)         # as requested
driver.GetSteeringController().SetGains(0.8, 0.0, 0.3)           # P-I-D (reasonable starting values)

# Speed (cruise control) PID gains
driver.GetSpeedController().SetGains(0.4, 0.0, 0.0)              # mainly proportional

driver.Initialize()

# ---------------------------------------------------------------------
# Irrlicht visualisation
# ---------------------------------------------------------------------
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('FEDA – ISO Double Lane-Change')
vis.SetWindowSize(1280, 1024)
# chase-camera tracks a point a little behind the car
vis.SetChaseCamera(chrono.ChVector3d(-3, 0, 1.1), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

print("VEHICLE MASS:", vehicle.GetVehicle().GetMass())

# ---------------------------------------------------------------------
# Simulation loop
# ---------------------------------------------------------------------
render_steps   = math.ceil(render_step_size / step_size)
step_number    = 0
realtime_timer = chrono.ChRealtimeStepTimer()

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    # Synchronise modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver.GetInputs(), terrain)
    vis.Synchronize(time, driver.GetInputs())

    # Advance simulation
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)