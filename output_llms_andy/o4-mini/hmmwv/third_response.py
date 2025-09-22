import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# ------------------------------------------------------------------------------
# 1) FIX: make sure the Chrono data path is set (this was redundant but harmless).
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# ------------------------------------------------------------------------------
# 2) Custom driver class
class MyDriver(veh.ChDriver):
    def __init__(self, vehicle, delay):
        super().__init__(vehicle)
        self.delay = delay
        # initialize the internal inputs struct to zero
        inp = self.GetInputs()
        inp.m_throttle = 0.0
        inp.m_steering = 0.0
        inp.m_braking = 0.0
        self.SetInputs(inp)

    def Synchronize(self, time):
        # Called every step: we compute throttle, steering, braking
        t = time - self.delay
        inp = self.GetInputs()

        # Throttle ramp: 0 -> 0.7 over 0.2 s, after delay
        if t <= 0.0:
            inp.m_throttle = 0.0
        elif t < 0.2:
            inp.m_throttle = 0.7 * (t / 0.2)
        else:
            inp.m_throttle = 0.7

        # Steering: sinusoidal starting at t = delay + 2.0
        if t > 2.0:
            # e.g. amplitude 0.5, angular freq = 1 rad/s
            inp.m_steering = 0.5 * math.sin(t - 2.0)
        else:
            inp.m_steering = 0.0

        # No braking in this scenario
        inp.m_braking = 0.0

        # push back to the base class
        self.SetInputs(inp)

# ------------------------------------------------------------------------------
# 3) Vehicle and environment setup (unchanged except driver)
# Initial vehicle position/orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

vis_type = veh.VisualizationType_PRIMITIVES
chassis_collision_type = veh.CollisionType_NONE
tire_model = veh.TireModelType_TMEASY
contact_method = chrono.ChContactMethod_NSC

step_size       = 1e-3
tire_step_size  = step_size
render_step_size = 1.0 / 50.0

# Create the vehicle
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

# Use the Bullet-based collision (optional)
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Rigid terrain
terrainLength = 100.0
terrainWidth  = 100.0

patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0,0,0), chrono.QUNIT),
    terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Irrlicht visualization
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo with Custom Driver')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(-3.0, 0.0, 1.1), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# ------------------------------------------------------------------------------
# 4) DRIVER SYSTEM REPLACEMENT
# Remove the interactive driver and plug in our MyDriver
driver = MyDriver(vehicle.GetVehicle(), delay=0.5)
driver.Initialize()

# ------------------------------------------------------------------------------
# Print total mass
print("VEHICLE MASS:", vehicle.GetVehicle().GetMass())

# Prepare stepping
render_steps = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

# ------------------------------------------------------------------------------
# 5) Main simulation loop with end condition at t = 4.0 s
while vis.Run():
    time = vehicle.GetSystem().GetChTime()
    if time >= 4.0:
        break

    # Render every render_steps
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    # Update all modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver.GetInputs(), terrain)
    vis.Synchronize(time, driver.GetInputs())

    # Advance one timestep
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)