import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# -------------------------------------------------------------------------
# Custom driver
# -------------------------------------------------------------------------
class MyDriver(veh.ChDriver):
    def __init__(self, vehicle, delay=0.5):
        veh.ChDriver.__init__(self, vehicle)

        self.delay = delay
        self.inputs = veh.DriverInputs()

        self.max_throttle = 0.7
        self.throttle_ramp_time = 0.2

        self.steering_start_time = 2.0
        self.steering_amplitude = 0.5
        self.steering_frequency = 0.5

    def Synchronize(self, time):
        # Apply input delay
        driver_time = time - self.delay

        if driver_time < 0:
            self.inputs.m_throttle = 0.0
            self.inputs.m_steering = 0.0
            self.inputs.m_braking = 0.0
            return

        # Throttle ramps from 0 to 0.7 over 0.2 seconds after the delay
        self.inputs.m_throttle = min(
            self.max_throttle,
            self.max_throttle * driver_time / self.throttle_ramp_time
        )

        # Steering follows a sinusoidal pattern starting at 2 seconds
        if driver_time >= self.steering_start_time:
            steering_time = driver_time - self.steering_start_time
            self.inputs.m_steering = self.steering_amplitude * math.sin(
                2.0 * math.pi * self.steering_frequency * steering_time
            )
        else:
            self.inputs.m_steering = 0.0

        # No braking command in this custom driver
        self.inputs.m_braking = 0.0

    def Advance(self, step):
        pass

    def GetInputs(self):
        return self.inputs


# -------------------------------------------------------------------------
# Initial vehicle location and orientation
# -------------------------------------------------------------------------
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts
vis_type = veh.VisualizationType_PRIMITIVES

# Collision type for chassis
chassis_collision_type = veh.CollisionType_NONE

# Tire model
tire_model = veh.TireModelType_TMEASY

# Terrain parameters
terrainHeight = 0.0
terrainLength = 100.0
terrainWidth = 100.0

# Point on chassis tracked by the camera
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Rendering frame rate
render_step_size = 1.0 / 50.0

# Simulation end time
end_time = 4.0

# -------------------------------------------------------------------------
# Create and initialize the HMMWV vehicle
# -------------------------------------------------------------------------
vehicle = veh.HMMWV_Full()
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

# -------------------------------------------------------------------------
# Create the terrain
# -------------------------------------------------------------------------
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(vehicle.GetSystem())

patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(
        chrono.ChVector3d(0, 0, terrainHeight),
        chrono.ChQuaterniond(1, 0, 0, 0)
    ),
    terrainLength,
    terrainWidth
)

patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

terrain.Initialize()

# -------------------------------------------------------------------------
# Create the Irrlicht visualization system
# -------------------------------------------------------------------------
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Custom Driver Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# -------------------------------------------------------------------------
# Create the custom driver system with 0.5 second delay
# -------------------------------------------------------------------------
driver = MyDriver(vehicle.GetVehicle(), delay=0.5)

# Output vehicle mass
print("VEHICLE MASS:", vehicle.GetVehicle().GetMass())

# Number of simulation steps between rendering frames
render_steps = math.ceil(render_step_size / step_size)

# Realtime timer
realtime_timer = chrono.ChRealtimeStepTimer()

step_number = 0
render_frame = 0

# -------------------------------------------------------------------------
# Simulation loop
# -------------------------------------------------------------------------
while vis.Run() and vehicle.GetSystem().GetChTime() < end_time:
    time = vehicle.GetSystem().GetChTime()

    # Render scene
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Update custom driver first, then retrieve inputs
    driver.Synchronize(time)
    driver_inputs = driver.GetInputs()

    # Synchronize modules
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1

    # Real-time pacing
    realtime_timer.Spin(step_size)