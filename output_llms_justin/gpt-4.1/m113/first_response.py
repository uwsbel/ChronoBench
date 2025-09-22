import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import time

# 1. Initialize PyChrono environment and core components
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# 2. Create the physical system
system = chrono.ChSystemSMC()

# 3. Create the M113 vehicle
initLoc = chrono.ChVectorD(0, 0, 0.5)  # Initial position (x, y, z)
initYaw = chrono.ChQuaternionD(1, 0, 0, 0)  # No initial rotation
vehicle = veh.M113_Vehicle(system, False, veh.ChassisCollisionType.NONE)
vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc, initYaw))
vehicle.Initialize()

# 4. Create the terrain
terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(
    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
    200, 200  # size x, y
)
patch.SetContactFrictionCoefficient(0.8)
patch.SetContactRestitutionCoefficient(0.01)
patch.SetMaterialSurface(veh.MaterialSurface.SMC)
patch.SetTexture(chrono.GetChronoDataFile("terrain/textures/grass.jpg"), 200, 200)
terrain.Initialize()

# 5. Create the driver system (keyboard driver)
driver = veh.ChDriverInputs()
driver_system = veh.ChIrrGuiDriver(vehicle, irr.KEY_UP, irr.KEY_DOWN, irr.KEY_LEFT, irr.KEY_RIGHT, irr.KEY_SPACE)
driver_system.SetThrottleDelta(0.02)
driver_system.SetSteeringDelta(0.02)
driver_system.SetBrakingDelta(0.1)
driver_system.Initialize()

# 6. Create the Irrlicht visualization system
vis = veh.ChWheeledVehicleIrrApp(vehicle, 'PyChrono M113 Demo', irr.dimension2du(1024,768))
vis.SetSkyBox()
vis.AddTypicalLights(chrono.ChVectorD(30, 100, 40), chrono.ChVectorD(-30, -100, 40), 250, 130)
vis.AddCamera(chrono.ChVectorD(0, 2, 1.5), chrono.ChVectorD(0, 0, 0.5))
vis.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 1.75), 6.0, 0.5)
vis.Initialize()

# 7. Simulation parameters
step_size = 2e-3
render_step_size = 1.0 / 50  # Render at 50 FPS
realtime_timer = chrono.ChRealtimeStepTimer()

# 8. Simulation loop
while vis.Run():
    # Input handling
    driver_inputs = driver_system.GetInputs()

    # Synchronize all modules
    time = system.GetChTime()
    driver_system.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize('M113 Simulation', driver_inputs)

    # Advance simulation for one timestep
    driver_system.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Render scene
    if vis.IsRendering():
        vis.BeginScene()
        vis.DrawAll()
        vis.EndScene()

    # Maintain real-time
    realtime_timer.Spin(step_size)

# Cleanup
vis.GetDevice().closeDevice()