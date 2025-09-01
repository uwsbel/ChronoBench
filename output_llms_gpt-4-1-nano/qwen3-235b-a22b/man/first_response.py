import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

# ----------------------------
# 1. Initialize the PyChrono System
# ----------------------------
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))  # Set gravity

# ----------------------------
# 2. Create the Rigid Terrain
# ----------------------------
terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(chrono.ChCoordsysD(), chrono.ChVectorD(200, 100, 1), True, True)
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 100)  # Customizable texture
terrain.Initialize()

# ----------------------------
# 3. Create and Configure the MAN 10t Truck
# ----------------------------
truck = veh.MAN_10t(system)
truck.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1.5), chrono.QUNIT))  # Start position and orientation

# Set visualization types
truck.SetChassisVisualizationType(veh.VisualizationType_MESH)
truck.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
truck.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
truck.SetWheelVisualizationType(veh.VisualizationType_MESH)

# Replace default tires with TMeasy models
for axle in truck.GetAxles():
    for wheel in axle.GetWheels():
        tire = veh.ChTMeasyTire("TMeasyTire")
        tire.SetSize(0.3, 0.2)  # Example tire dimensions
        tire.SetRimRadius(0.3)
        tire.SetTireWidth(0.2)
        tire.SetContactMaterialSurface(chrono.ChMaterialSurfaceNSC())
        tire.Initialize(wheel, veh.WheelID())  # Attach to the wheel

# ----------------------------
# 4. Set Up Irrlicht Visualization
# ----------------------------
application = irr.ChIrrApp(system, 'MAN 10t Truck Simulation', irr.dimension2du(1024, 768))
application.AddTypicalLogo()
application.AddSkyBox()
application.AddCamera(chrono.ChVectorD(0, -5, 2), chrono.ChVectorD(0, 0, 0))  # Initial camera position
application.AddLightWithShadow(chrono.ChVectorD(10, -25, 10), chrono.ChVectorD(0, 0, 0), 100, 10, 20, 60)

# Set up chase camera
chase_camera = veh.ChChaseCamera(truck.GetChassisBody(), 4.0, 0.5)  # Distance and height
application.SetChaseCamera(chase_camera)

# ----------------------------
# 5. Set Up Real-Time Driver Controls
# ----------------------------
driver = veh.ChIrrGuiDriver(application.GetDevice())
driver.SetSteeringDelta(0.05)
driver.SetThrottleDelta(0.05)
driver.SetBrakingDelta(0.05)

# ----------------------------
# 6. Simulation Loop
# ----------------------------
step_size = 1e-3  # Simulation time step
rt_timer = chrono.ChRealtimeStepTimer()

while application.GetDevice().run():
    time = system.GetChTime()
    driver_inputs = driver.GetInputs()

    # Synchronize components
    truck.Synchronize(time, driver_inputs)
    terrain.Synchronize(time)
    application.Synchronize("MAN 10t Truck Simulation", driver_inputs)

    # Advance the simulation
    system.DoStepDynamics(step_size)

    # Update visualization
    application.BeginScene()
    application.DrawAll()
    application.EndScene()

    rt_timer.Spin(step_size)