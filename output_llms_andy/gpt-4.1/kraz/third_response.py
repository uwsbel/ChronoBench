import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set Chrono data paths
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# --- Initial locations and orientations ---
# Truck (Kraz)
initLoc_truck = chrono.ChVector3d(-10, 0, 0.5)  # Changed from (0,0,0.5)
initRot_truck = chrono.ChQuaterniond(1, 0, 0, 0)  # Identity quaternion

# Sedan
initLoc_sedan = chrono.ChVector3d(0, 3, 0.5)  # Offset to avoid collision
initRot_sedan = chrono.ChQuaterniond(1, 0, 0, 0)  # Identity quaternion

# Visualization type for vehicle parts
vis_type = veh.VisualizationType_MESH

# Collision type for chassis
chassis_collision_type = veh.CollisionType_NONE

# Tire model types
tire_model_truck = veh.TireModelType_RIGID
tire_model_sedan = veh.TireModelType_TMEASY

# Terrain parameters
terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0

# Camera tracking point
trackPoint = chrono.ChVector3d(0, 0, 2.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# ------------------ Create the Kraz vehicle (truck) ------------------
vehicle_truck = veh.Kraz()
vehicle_truck.SetContactMethod(contact_method)
vehicle_truck.SetChassisCollisionType(chassis_collision_type)
vehicle_truck.SetChassisFixed(False)
vehicle_truck.SetInitPosition(chrono.ChCoordsysd(initLoc_truck, initRot_truck))
vehicle_truck.SetTireType(tire_model_truck)
vehicle_truck.Initialize()

vehicle_truck.SetChassisVisualizationType(vis_type)
vehicle_truck.SetSteeringVisualizationType(vis_type)
vehicle_truck.SetSuspensionVisualizationType(vis_type)
vehicle_truck.SetWheelVisualizationType(vis_type)
vehicle_truck.SetTireVisualizationType(vis_type)

vehicle_truck.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# ------------------ Create the Sedan vehicle ------------------
vehicle_sedan = veh.Sedan()
vehicle_sedan.SetContactMethod(contact_method)
vehicle_sedan.SetChassisCollisionType(chassis_collision_type)
vehicle_sedan.SetChassisFixed(False)
vehicle_sedan.SetInitPosition(chrono.ChCoordsysd(initLoc_sedan, initRot_sedan))
vehicle_sedan.SetTireType(tire_model_sedan)
vehicle_sedan.Initialize()

vehicle_sedan.SetChassisVisualizationType(vis_type)
vehicle_sedan.SetSteeringVisualizationType(vis_type)
vehicle_sedan.SetSuspensionVisualizationType(vis_type)
vehicle_sedan.SetWheelVisualizationType(vis_type)
vehicle_sedan.SetTireVisualizationType(vis_type)

# Both vehicles share the same Chrono system
system = vehicle_truck.GetSystem()

# ------------------ Create the terrain using a highway mesh ------------------
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(system)
# Use a predefined highway mesh (ensure the file exists in your data path)
highway_mesh_file = veh.GetDataFile("terrain/Highway/highway.obj")
patch = terrain.AddPatch(patch_mat, highway_mesh_file, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
terrain.Initialize()

# ------------------ Create the Irrlicht visualization system ------------------
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Kraz and Sedan Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 25.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle_truck.GetTractor())
vis.AttachVehicle(vehicle_sedan.GetVehicle())  # Attach sedan for visualization

# ------------------ Create the driver systems ------------------
# Truck: Interactive driver
driver_truck = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver_truck.SetSteeringDelta(render_step_size / steering_time)
driver_truck.SetThrottleDelta(render_step_size / throttle_time)
driver_truck.SetBrakingDelta(render_step_size / braking_time)
driver_truck.Initialize()

# Sedan: Simple driver with fixed throttle and steering
class FixedDriver(veh.ChDriver):
    def __init__(self, vehicle, throttle=0.5, steering=0.0, braking=0.0):
        super().__init__(vehicle.GetVehicle())
        self.inputs = veh.DriverInputs()
        self.inputs.m_throttle = throttle
        self.inputs.m_steering = steering
        self.inputs.m_braking = braking
    def Synchronize(self, time):
        pass
    def Advance(self, step):
        pass
    def GetInputs(self):
        return self.inputs

fixed_throttle = 0.6
fixed_steering = 0.0
fixed_braking = 0.0
driver_sedan = FixedDriver(vehicle_sedan, throttle=fixed_throttle, steering=fixed_steering, braking=fixed_braking)

# Output vehicle masses
print("TRUCK MASS: ", vehicle_truck.GetTractor().GetMass())
print("SEDAN MASS: ", vehicle_sedan.GetVehicle().GetMass())

# Number of simulation steps between render events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counters
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

# --- State storage for truck's tractor and trailer ---
truck_tractor_states = []
truck_trailer_states = []

# Main simulation loop
while vis.Run():
    time = system.GetChTime()

    # Render scene
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Get driver inputs
    driver_inputs_truck = driver_truck.GetInputs()
    driver_inputs_sedan = driver_sedan.GetInputs()

    # Synchronize modules
    driver_truck.Synchronize(time)
    driver_sedan.Synchronize(time)
    terrain.Synchronize(time)
    vehicle_truck.Synchronize(time, driver_inputs_truck, terrain)
    vehicle_sedan.Synchronize(time, driver_inputs_sedan, terrain)
    vis.Synchronize(time, driver_inputs_truck)  # Pass truck inputs for camera

    # Advance simulation for one timestep for all modules
    driver_truck.Advance(step_size)
    driver_sedan.Advance(step_size)
    terrain.Advance(step_size)
    vehicle_truck.Advance(step_size)
    vehicle_sedan.Advance(step_size)
    vis.Advance(step_size)

    # --- Store truck's tractor and trailer states ---
    tractor = vehicle_truck.GetTractor()
    trailer = vehicle_truck.GetTrailer()
    tractor_state = {
        'time': time,
        'pos': tractor.GetPos(),
        'rot': tractor.GetRot(),
        'lin_vel': tractor.GetPos_dt(),
        'ang_vel': tractor.GetWvel_loc()
    }
    truck_tractor_states.append(tractor_state)
    if trailer:
        trailer_state = {
            'time': time,
            'pos': trailer.GetPos(),
            'rot': trailer.GetRot(),
            'lin_vel': trailer.GetPos_dt(),
            'ang_vel': trailer.GetWvel_loc()
        }
        truck_trailer_states.append(trailer_state)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)

# (Optional) Save or process truck_tractor_states and truck_trailer_states as needed