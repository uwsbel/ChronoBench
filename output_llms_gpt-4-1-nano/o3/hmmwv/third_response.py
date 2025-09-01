import math
import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh


# ------------------------------------------------
# 1. Chrono/Vehicle initialisation
# ------------------------------------------------
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# Vehicle initial pose
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualisation and collision options
vis_type              = veh.VisualizationType_PRIMITIVES
chassis_collision_type = veh.ChassisCollisionType_NONE      # <- fixed wrong enum name
tire_model            = veh.TireModelType_TMEASY            # rigid or TMeasy

# Terrain parameters
terrainHeight = 0
terrainLength = 100.0
terrainWidth  = 100.0

# Camera tracking point (relative to chassis COM)
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)

# Contact method & solver step
contact_method    = chrono.ChContactMethod_NSC
step_size         = 1e-3
tire_step_size    = step_size
render_step_size  = 1.0 / 50.0         # 50 FPS

# ------------------------------------------------
# 2. Create the HMMWV vehicle
# ------------------------------------------------
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

# Use Bullet collision system
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# ------------------------------------------------
# 3. Rigid-terrain definition
# ------------------------------------------------
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrainHeight),
                       chrono.ChQuaterniond(1, 0, 0, 0)),     # changed: replaced non-existing chrono.QUNIT
    terrainLength,
    terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# ------------------------------------------------
# 4. Irrlicht visualisation
# ------------------------------------------------
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV Demo")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# ------------------------------------------------
# 5. Custom driver class
# ------------------------------------------------
class MyDriver(veh.ChDriver):
    """
    Simple scripted driver with:
        • initial 'delay' (no inputs)
        • throttle linearly ramping to 0.7 in 0.2 s
        • sinusoidal steering starting at 2 s
    """
    def __init__(self, chassis, delay=0.0):
        super().__init__(chassis)          # initialise base class
        self.delay  = delay

    # Called once every time step from the main loop
    def Synchronize(self, time):
        # Default values
        throttle = 0.0
        steering = 0.0
        braking  = 0.0

        # After delay – throttle ramp
        if time >= self.delay:
            t = time - self.delay
            if t < 0.2:
                throttle = 0.7 * (t / 0.2)
            else:
                throttle = 0.7

        # Steering: sinusoid from t = 2 s
        if time >= 2.0:
            steering = 0.5 * math.sin(2.0 * math.pi * (time - 2.0))

        # Store in the base-class container
        self.SetThrottle(throttle)
        self.SetSteering(steering)
        self.SetBraking(braking)

    # Nothing special to do every step, but keep function for completeness
    def Advance(self, step):
        pass


# Instantiate our driver with the requested 0.5-s delay
driver = MyDriver(vehicle.GetVehicle(), delay=0.5)
driver.Initialize()

print("VEHICLE MASS:", vehicle.GetVehicle().GetMass())

# ------------------------------------------------
# 6. Simulation loop (stops automatically at 4 s)
# ------------------------------------------------
render_steps   = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()

step_number  = 0
render_frame = 0

while vis.Run() and vehicle.GetSystem().GetChTime() < 4.0:

    time = vehicle.GetSystem().GetChTime()

    # Render once every 'render_step_size'
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Driver inputs & module synchronisation
    driver.Synchronize(time)               # custom behaviour
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver.GetInputs(), terrain)
    vis.Synchronize(time, driver.GetInputs())

    # Advance all subsystems
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Real-time pace
    realtime_timer.Spin(step_size)

    step_number += 1