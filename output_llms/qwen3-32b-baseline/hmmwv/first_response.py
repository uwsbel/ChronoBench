import pychrono as chrono
import pychrono.vehicle as vehicle
import pychrono.irrlicht as irr

# 1. Initialize the PyChrono environment and core components
sys = chrono.ChSystemNSC()
sys.Set_Gravity(chrono.ChVectorD(0, -9.81, 0))

# 2. Create the HMMWV vehicle
hmmwv = vehicle.hmmwv.ChHMMWV_Vehicle(sys)
hmmwv.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.ChQuaternionD(1, 0, 0, 0)))
hmmwv.SetChassisVisualizationType(vehicle.VisualizationType.PRIMITIVES)
hmmwv.SetSuspensionVisualizationType(vehicle.VisualizationType.PRIMITIVES)
hmmwv.SetSteeringVisualizationType(vehicle.VisualizationType.PRIMITIVES)
hmmwv.SetWheelVisualizationType(vehicle.VisualizationType.PRIMITIVES)
hmmwv.Initialize()

# Set TMEASY tire model for all wheels
for axle in hmmwv.GetAxles():
    for side in [vehicle.ChAxle.Side.LEFT, vehicle.ChAxle.Side.RIGHT]:
        wheel = axle.GetWheel(side)
        tire = wheel.GetTire()
        tire.SetType(vehicle.ChTire.Type.TMEASY)

# 3. Create the rigid terrain
terrain = vehicle.terrain.RigidTerrain(sys)
patch = terrain.AddPatch(chrono.ChCoordsysD(), 100.0, 100.0)
patch.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"), 100, 100)
terrain.Initialize()

# 4. Set up Irrlicht visualization
app = irr.ChIrrApp(sys, 'HMMWV Simulation', irr.dimension2du(800, 600))
app.SetSkyBox()
app.SetCamera(chrono.ChVectorD(0, 0, 1.5), chrono.ChVectorD(0, 0, 0))
app.SetLightDirection(chrono.ChVectorD(0, -1, -1))
app.AddTypicalLogo(chrono.GetChronoDataFile('logo_pychrono.png'))
app.SetTargetFPS(50)

# Add vehicle and terrain to visualization
app.AddItem(hmmwv.GetChassisBody())
app.AddItem(terrain.GetMesh())

# 5. Configure interactive driver system
driver = vehicle.ChIrrGuiDriver(app)
driver.SetSteeringClamp(1.0)
driver.SetThrottleClamp(1.0)
driver.SetBrakingClamp(1.0)
driver.Initialize()

# 6. Simulation loop
time_step = 1 / 50
while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()
    
    # Update driver inputs
    driver.Update()
    hmmwv.SetSteering(driver.GetSteering())
    hmmwv.SetThrottle(driver.GetThrottle())
    hmmwv.SetBraking(driver.GetBraking())
    
    # Advance simulation
    sys.DoStepDynamics(time_step)
    
    app.EndScene()