import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

init_loc = chrono.ChVector3d(2.0, -60.0, 0.5)                        # HMMWV spawn on the highway (road runs along Y)
init_rot = chrono.QuatFromAngleZ(math.pi / 2)                        # heading along +Y (down the highway)
step_size = 2e-3                                                     # dynamics step (s)
tire_step_size = 1e-3                                                # tire substep (s)

hmmwv = veh.HMMWV_Full()                                             # full HMMWV catalog model
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)                   # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)                # no chassis collision shell
hmmwv.SetChassisFixed(False)                                         # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))        # location + orientation
hmmwv.SetTireType(veh.TireModelType_TMEASY)                          # prompt: TMEASY tire model
hmmwv.SetTireStepSize(tire_step_size)                                # tire integration substep
hmmwv.Initialize()                                                   # build the vehicle subsystems

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)        # mesh visuals for all components
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

system = hmmwv.GetSystem()                                           # wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED, after Initialize
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())               # report total vehicle mass

terrain = veh.RigidTerrain(system)                                   # rigid custom-mesh terrain
patch_mat = chrono.ChContactMaterialNSC()                            # NSC patch material
patch_mat.SetFriction(0.9)                                           # tire-road friction
patch_mat.SetRestitution(0.01)                                       # nearly inelastic road
patch = terrain.AddPatch(                                            # collision mesh patch
    patch_mat,
    chrono.CSYSNORM,                                                 # mesh in world coords
    veh.GetDataFile("terrain/meshes/Highway_col.obj"),              # custom collision mesh
    True,                                                            # connected mesh
    0.01,                                                            # sweep sphere radius — thickens thin road mesh for stable contact
    False,                                                           # no auto visualization (use vis mesh)
)

vis_mesh = chrono.ChTriangleMeshConnected()                          # custom visual mesh
vis_mesh.LoadWavefrontMesh(veh.GetDataFile("terrain/meshes/Highway_vis.obj"), True, True)
vis_shape = chrono.ChVisualShapeTriangleMesh()                       # visual shape for the road
vis_shape.SetMesh(vis_mesh)
vis_shape.SetName("highway_vis")
vis_shape.SetMutable(False)                                          # static road geometry
road_body = chrono.ChBody()                                          # carrier body for the visual mesh
road_body.SetPos(chrono.ChVector3d(0, 0, 0))
road_body.SetFixed(True)                                             # road never moves
road_body.AddVisualShape(vis_shape, chrono.ChFramed())
system.AddBody(road_body)

terrain.Initialize()                                                 # finalize terrain

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                     # vehicle Irrlicht window
vis.SetWindowTitle("HMMWV on Highway")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)          # chase camera on chassis
vis.Initialize()                                                     # device first, then scene elements
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                                            # vehicle demos use a directional light
vis.AttachVehicle(hmmwv.GetVehicle())                               # bind chassis/wheel/tire visuals

driver = veh.ChInteractiveDriverIRR(vis)                             # interactive steering/throttle/braking
render_step_size = 1.0 / 50.0                                        # 50 fps render cadence
steering_time = 1.0                                                  # s to reach full steering
throttle_time = 1.0                                                  # s to reach full throttle
braking_time = 0.3                                                   # s to reach full braking
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

render_steps = math.ceil(render_step_size / step_size)              # physics steps per rendered frame
realtime_timer = chrono.ChRealtimeStepTimer()                       # wall-clock pacing
step_number = 0
while vis.Run():
    time = hmmwv.GetSystem().GetChTime()                            # current sim time

    if step_number % render_steps == 0:                            # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()


    driver_inputs = driver.GetInputs()                             # current steering/throttle/braking

    driver.Synchronize(time)                                       # advance subsystem inputs
    terrain.Synchronize(time)
    hmmwv.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    driver.Advance(step_size)                                      # integrate one step
    terrain.Advance(step_size)
    hmmwv.Advance(step_size)                                       # advances the wrapper-owned system
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)                                # spin so wall-clock matches sim time
