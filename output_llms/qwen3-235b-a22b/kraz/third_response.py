import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set data paths
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle locations and orientations
initLoc_truck = chrono.ChVector3d(0, -2, 0.5)
initRot_truck = chrono.ChQuaterniond(1, 0, 0, 0)
initLoc_sedan = chrono.ChVector3d(0, 2, 0.5)
initRot_sedan = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type
vis_type = veh.VisualizationType_MESH

# Collision type
chassis_collision_type = veh.CollisionType_NONE

# Tire models
tire_model_truck = veh.TireModelType_RIGID
tire_model_sedan = veh.TireModelType_TMEASY

# Contact method
contact_method = chrono.ChContactMethod_NSC

# Simulation step sizes
step_size = 1e-3
render_step_size = 1.0 / 50

# Camera tracking point for truck
trackPoint = chrono.ChVector3d(0, 0, 2.1)

# Create the Kraz truck
truck = veh.Kraz()
truck.SetContactMethod(contact_method)
truck.SetChassisCollisionType(chassis_collision_type)
truck.SetChassisFixed(False)
truck.SetInitPosition(chrono.ChCoordsysd(initLoc_truck, initRot_truck))
truck.SetTireType(tire_model_truck)  # Corrected: Added tire type setting
truck.Initialize()
truck.SetChassisVisualizationType(vis_type, vis_type)
truck.SetSteeringVisualizationType(vis_type)
truck.SetSuspensionVisualizationType(vis_type, vis_type)
truck.SetWheelVisualizationType(vis_type, vis_type)
truck.SetTireVisualizationType(vis_type, vis_type)
truck.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the Sedan
sedan = veh.Sedan()
sedan.SetContactMethod(contact_method)
sedan.SetChassisCollisionType(chassis_collision_type)
sedan.SetChassisFixed(False)
sedan.SetInitPosition(chrono.ChCoordsysd(initLoc_sedan, initRot_sedan))
sedan.SetTireType(tire_model_sedan)
sedan.Initialize()
sedan.SetChassisVisualizationType(vis_type, vis_type)
sedan.SetSuspensionVisualizationType(vis_type, vis_type)
sedan.SetSteeringVisualizationType(vis_type)
sedan.SetWheelVisualizationType(vis_type, vis_type)
sedan.SetTireVisualizationType(vis_type, vis_type)

# Create the terrain with highway mesh
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(truck.GetSystem())
highway_mesh = veh.GetDataFile("terrain/meshes/highway.obj")
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), highway_mesh)
patch.SetTexture(veh.GetDataFile("terrain/textures/highway.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
terrain.Initialize()

# Create visualization system
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Two Vehicle Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 25.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(truck.GetTractor())

# Create driver for truck
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# Create driver for sedan with fixed inputs
sedan_driver = veh.ChDriver(sedan.GetVehicle())
sedan_driver.Initialize()

# Output vehicle masses
print("TRUCK MASS:", truck.GetTractor().GetMass())
print("SEDAN MASS:", sedan.GetVehicle().GetMass())

# State logging file
state_file = open("truck_states.txt", "w")
state_file.write("Time Tractor_PosX Tractor_PosY Tractor_PosZ Tractor_QuatW Tractor_QuatX Tractor_QuatY Tractor_QuatZ Trailer_PosX Trailer_PosY Trailer_PosZ Trailer_QuatW Trailer_QuatX Trailer_QuatY Trailer_QuatZ\n")

# Simulation loop
render_steps = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

while vis.Run():
    time = truck.GetSystem().GetChTime()

    # Render scene and log data
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        
        # Log truck states
        tractor = truck.GetTractor()
        trailer = truck.GetTrailer()
        t_pos = tractor.GetChassisBody().GetPos()
        t_rot = tractor.GetChassisBody().GetRot()
        tr_pos = trailer.GetChassisBody().GetPos()
        tr_rot = trailer.GetChassisBody().GetRot()
        state_file.write(f"{time} ")
        state_file.write(f"{t_pos.x} {t_pos.y} {t_pos.z} ")
        state_file.write(f"{t_rot.e0} {t_rot.e1} {t_rot.e2} {t_rot.e3} ")
        state_file.write(f"{tr_pos.x} {tr_pos.y} {tr_pos.z} ")
        state_file.write(f"{tr_rot.e0} {tr_rot.e1} {tr_rot.e2} {tr_rot.e3}\n")
        state_file.flush()

    # Get and set driver inputs
    driver_inputs = driver.GetInputs()
    sedan_driver.Synchronize(time)
    sedan_inputs = sedan_driver.GetInputs()
    sedan_inputs.m_throttle = 0.7
    sedan_inputs.m_steering = 0.0
    sedan_inputs.m_braking = 0.0

    # Synchronize all modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    truck.Synchronize(time, driver_inputs, terrain)
    sedan.Synchronize(time, sedan_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    truck.Advance(step_size)
    sedan.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)

# Clean up
state_file.close()