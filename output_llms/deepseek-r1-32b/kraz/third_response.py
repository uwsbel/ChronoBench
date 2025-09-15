import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set Chrono data paths
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Truck parameters
truck_initLoc = chrono.ChVector3d(0, 0, 0.5)
truck_initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Sedan parameters
sedan_initLoc = chrono.ChVector3d(10, 0, 0.5)
sedan_initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization and collision settings
vis_type = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
render_step_size = 1.0 / 50  # FPS = 50

# Create truck
truck = veh.Kraz()
truck.SetContactMethod(contact_method)
truck.SetChassisCollisionType(chassis_collision_type)
truck.SetChassisFixed(False)
truck.SetInitPosition(chrono.ChCoordsysd(truck_initLoc, truck_initRot))
truck.Initialize()

# Set truck visualization
truck.SetChassisVisualizationType(vis_type, vis_type)
truck.SetSteeringVisualizationType(vis_type)
truck.SetSuspensionVisualizationType(vis_type, vis_type)
truck.SetWheelVisualizationType(vis_type, vis_type)
truck.SetTireVisualizationType(vis_type, vis_type)
truck.GetTires().SetTireType(veh.TireModelType_RIGID)

# Create sedan
sedan = veh.Sedan()
sedan.SetContactMethod(contact_method)
sedan.SetChassisCollisionType(chassis_collision_type)
sedan.SetChassisFixed(False)
sedan.SetInitPosition(chrono.ChCoordsysd(sedan_initLoc, sedan_initRot))
sedan.Initialize()

# Set sedan visualization
sedan.SetChassisVisualizationType(vis_type, vis_type)
sedan.SetSteeringVisualizationType(vis_type)
sedan.SetSuspensionVisualizationType(vis_type, vis_type)
sedan.SetWheelVisualizationType(vis_type, vis_type)
sedan.SetTireVisualizationType(vis_type, vis_type)

# Create terrain with highway mesh
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(truck.GetSystem())
terrain.SetVisualShape(veh.GetDataFile('terrain/textures/highway.obj'), 100, 100)
terrain.Initialize()

# Create visualization
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Truck and Sedan Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0,0,2.1), 25.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(truck.GetTractor())
vis.AttachVehicle(sedan)

# Create drivers
truck_driver = veh.ChInteractiveDriverIRR(vis)
truck_driver.SetSteeringDelta(render_step_size / 1.0)
truck_driver.SetThrottleDelta(render_step_size / 1.0)
truck_driver.SetBrakingDelta(render_step_size / 0.3)
truck_driver.Initialize()

sedan_driver = veh.ChDriver(sedan.GetVehicle())
sedan_driver.SetThrottle(0.5)
sedan_driver.SetSteering(0.0)

# Store truck state
truck_states = []

# Simulation loop
render_steps = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = truck.GetSystem().GetChTime()
    
    # Store truck state
    tractor = truck.GetTractor()
    truck_states.append({
        'time': time,
        'position': tractor.GetPos(),
        'velocity': tractor.GetPos_dt(),
        'angular_velocity': tractor.GetWvel_par()
    })
    
    # Render scene
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1
    
    # Update modules
    truck_driver.Synchronize(time)
    terrain.Synchronize(time)
    truck.Synchronize(time, truck_driver.GetInputs(), terrain)
    sedan.Synchronize(time, sedan_driver.GetInputs(), terrain)
    vis.Synchronize(time, truck_driver.GetInputs())
    
    # Advance simulation
    truck_driver.Advance(step_size)
    terrain.Advance(step_size)
    truck.Advance(step_size)
    sedan.Advance(step_size)
    vis.Advance(step_size)
    
    # Increment frame number
    step_number += 1
    
    # Spin for real time
    realtime_timer.Spin(step_size)