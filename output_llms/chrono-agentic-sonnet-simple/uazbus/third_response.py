import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

chrono.SetChronoDataPath(chrono.GetChronoDataPath())              # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')          # locate vehicle data files

step_size = 1e-3                                                   # simulation time step (s)
sim_end = 20.0                                                     # total simulation time (s)
render_fps = 50.0                                                  # frames per second for rendering
render_every = max(1, round(1.0 / (render_fps * step_size)))      # steps between rendered frames

# --- UAZBUS vehicle ---
uaz = veh.UAZBUS()                                                 # UAZ bus catalog wrapper
uaz.SetContactMethod(chrono.ChContactMethod_NSC)                  # NSC for rigid terrain
uaz.SetChassisCollisionType(veh.CollisionType_NONE)               # no chassis collision mesh
uaz.SetChassisFixed(False)                                        # MANDATORY — must not be fixed
uaz.SetInitPosition(chrono.ChCoordsysd(
    chrono.ChVector3d(0, 0, 0.5),                                 # spawn slightly above ground
    chrono.QUNIT))
uaz.SetTireType(veh.TireModelType_RIGID)                          # prompt: rigid tire model
uaz.SetTireStepSize(step_size)
uaz.Initialize()

system = uaz.GetSystem()                                          # take system from wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact scene

print("VEHICLE MASS: ", uaz.GetVehicle().GetMass())               # truth's literal banner

# Set visualization types
uaz.SetChassisVisualizationType(veh.VisualizationType_MESH)
uaz.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
uaz.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
uaz.SetWheelVisualizationType(veh.VisualizationType_MESH)
uaz.SetTireVisualizationType(veh.VisualizationType_MESH)

# --- Rigid terrain ---
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()                         # NSC — matches vehicle contact method
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,                                              # flat ground at origin
    200.0, 200.0)                                                 # large flat patch
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# --- Box obstacle ---
box_mat = chrono.ChContactMaterialNSC()                           # NSC contact material for obstacle
box_mat.SetFriction(0.9)
box_mat.SetRestitution(0.01)
box_body = chrono.ChBodyEasyBox(0.5, 5.0, 0.2, 1000, True, True, box_mat)  # dims: 0.5x5x0.2 m
box_body.SetPos(chrono.ChVector3d(5, 0, 0.1))                    # position per prompt
box_body.SetFixed(True)                                           # fixed in place per prompt
system.AddBody(box_body)

# --- Irrlicht visualization (vehicle-specific) ---
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("UAZBUS - Rigid Tires + Obstacle")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)    # chase camera behind vehicle
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                                         # directional light for vehicle truths
vis.AttachVehicle(uaz.GetVehicle())

# --- Interactive driver (scored core default) ---
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_every * step_size / 1.0)          # ramp time 1 s
driver.SetThrottleDelta(render_every * step_size / 1.0)          # ramp time 1 s
driver.SetBrakingDelta(render_every * step_size / 0.3)           # ramp time 0.3 s
driver.Initialize()

# --- Simulation loop ---
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0


while vis.Run():
    time = system.GetChTime()

    if step_number % render_every == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()
    driver.SetThrottle(0.5)                                       # constant throttle per prompt

    driver.Synchronize(time)
    terrain.Synchronize(time)
    uaz.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)


    driver.Advance(step_size)
    terrain.Advance(step_size)
    uaz.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)

    if time >= sim_end:                                           # cap run at sim_end
        break
