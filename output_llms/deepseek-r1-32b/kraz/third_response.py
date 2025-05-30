import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set Chrono data paths
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle locations and orientations
truck_initLoc = chrono.ChVector3d(-5, 0, 1)  # Moved back to make space for sedan
truck_initRot = chrono.ChQuaterniond(1, 0, 0, 0)

sedan_initLoc = chrono.ChVector3d(5, 0, 1)   # Placed in front of truck
sedan_initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization types
vis_type = veh.VisualizationType_MESH

# Collision types
chassis_collision_type = veh.CollisionType_NONE

# Tire models
truck_tire_model = veh.TireModelType_RIGID  # Changed to RIGID for truck
sedan_tire_model = veh.TireModelType_TMEASY

# Terrain parameters
terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0

# Camera tracking points
truck_trackPoint = chrono.ChVector3d(0, 0, 2.1)
sedan_trackPoint = chrono.ChVector3d(0, 0, 1.5)

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

truck.SetChassisVisualizationType(vis_type)
truck.SetSteeringVisualizationType(vis_type)
truck.SetSuspensionVisualizationType(vis_type)
truck.SetWheelVisualizationType(vis_type)
truck.SetTireVisualizationType(vis_type)

# Set tire model for truck
truck.SetTireType(truck_tire_model)

# Create sedan
sedan = veh.Sedan()
sedan.SetContactMethod(contact_method)
sedan.SetChassisCollisionType(chassis_collision_type)
sedan.SetChassisFixed(False)
sedan.SetInitPosition(chrono.ChCoordsysd(sedan_initLoc, sedan_initRot))
sedan.Initialize()

sedan.SetChassisVisualizationType(vis_type)
sedan.SetSteeringVisualizationType(vis_type)
sedan.SetSuspensionVisualizationType(vis_type)
sedan.SetWheelVisualizationType(vis_type)
sedan.SetTireVisualizationType(vis_type)

# Set tire model for sedan
sedan.SetTireType(sedan_tire_model)

# Set up collision system
truck.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sedan.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create terrain with highway texture
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(truck.GetSystem())
patch = terrain.AddPatch(patch_mat,
                        chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                        terrainLength, terrainWidth)

patch.SetTexture(veh.GetDataFile("terrain/textures/highway.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create visualization
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Truck and Sedan Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(truck_trackPoint, 25.0, 1.5)
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

sedan_driver = veh.ChDriverAIWheeled()
sedan_driver.SetThrottle(0.8)  # Fixed throttle for forward movement
sedan_driver.SetSteering(0.0)  # Straight movement

# Output vehicle masses
print("TRUCK MASS:", truck.GetTractor().GetMass())
print("SEDAN MASS:", sedan.GetMass())

# Initialize simulation variables
render_steps = math.ceil(render_step_size / step_size)
step_number = 0
render_frame = 0

# Data storage for truck state
truck_tractor_states = []
truck_trailer_states = []

while vis.Run():
    time = truck.GetSystem().GetChTime()

    # Render scene
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1
        
        # Store truck state
        tractor = truck.GetTractor()
        trailer = truck.GetTrailer()
        truck_tractor_states.append((tractor.GetPos(), tractor.GetRot()))
        truck_trailer_states.append((trailer.GetPos(), trailer.GetRot()))

    # Update modules
    truck_driver.Synchronize(time)
    sedan_driver.Synchronize(time)
    terrain.Synchronize(time)
    truck.Synchronize(time, truck_driver.GetInputs(), terrain)
    sedan.Synchronize(time, sedan_driver.GetInputs(), terrain)
    vis.Synchronize(time, truck_driver.GetInputs())

    # Advance simulation
    truck_driver.Advance(step_size)
    sedan_driver.Advance(step_size)
    terrain.Advance(step_size)
    truck.Advance(step_size)
    sedan.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Spin for real time
    realtime_timer.Spin(step_size)

# Save truck state data
import pickle
with open('truck_state_data.pkl', 'wb') as f:
    pickle.dump({'tractor': truck_tractor_states, 'trailer': truck_trailer_states}, f)