import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr






chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")


contact_method = chrono.ChContactMethod_NSC
step_size = 1e-3
render_step_size = 1.0 / 50.0
render_steps = int(render_step_size / step_size)


init_loc = chrono.ChVector3d(0.0, 0.0, 0.6)
init_rot = chrono.QuatFromAngleZ(0.0)
init_fwd_vel = 0.0


terrain_length = 200.0
terrain_width = 200.0
terrain_friction = 0.9
terrain_restitution = 0.01


track_point = chrono.ChVector3d(0.0, 0.0, 1.0)
camera_distance = 8.0
camera_height = 1.5






m113 = veh.M113()

m113.SetContactMethod(contact_method)
m113.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))


if hasattr(m113, "SetInitFwdVel"):
    m113.SetInitFwdVel(init_fwd_vel)


m113.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)
m113.SetDrivelineType(veh.DrivelineTypeTV_BDS)
m113.SetEngineType(veh.EngineModelType_SIMPLE)
m113.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
m113.SetBrakeType(veh.BrakeType_SIMPLE)


m113.Initialize()


m113.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
m113.SetSprocketVisualizationType(veh.VisualizationType_PRIMITIVES)
m113.SetIdlerVisualizationType(veh.VisualizationType_PRIMITIVES)
m113.SetIdlerWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
m113.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
m113.SetRoadWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
m113.SetTrackShoeVisualizationType(veh.VisualizationType_PRIMITIVES)


system = m113.GetSystem()


system.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, -9.81))
system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
system.GetSolver().AsIterative().SetMaxIterations(150)


m113.GetVehicle().EnableRealtime(True)






terrain = veh.RigidTerrain(system)

terrain_mat = chrono.ChContactMaterialNSC()
terrain_mat.SetFriction(terrain_friction)
terrain_mat.SetRestitution(terrain_restitution)

terrain_patch = terrain.AddPatch(
    terrain_mat,
    chrono.ChCoordsysd(
        chrono.ChVector3d(0.0, 0.0, 0.0),
        chrono.QUNIT
    ),
    terrain_length,
    terrain_width
)

terrain_patch.SetTexture(
    veh.GetDataFile("terrain/textures/tile4.jpg"),
    terrain_length / 2.0,
    terrain_width / 2.0
)

terrain.Initialize()






vis = veh.ChTrackedVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("PyChrono M113 Tracked Vehicle")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(track_point, camera_distance, camera_height)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(m113.GetVehicle())






driver = veh.ChInteractiveDriverIRR(vis)


steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3

driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()






realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

while vis.Run():
    time = system.GetChTime()

    
    driver_inputs = driver.GetInputs()

    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    m113.Synchronize(time, driver_inputs)
    vis.Synchronize(time, driver_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    m113.Advance(step_size)
    vis.Advance(step_size)

    
    realtime_timer.Spin(step_size)

    step_number += 1