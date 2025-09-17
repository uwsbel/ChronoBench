import pychrono as chrono
import pychrono.irrlicht as irr
from pychrono import vehicle as veh

# Initialize Chrono system
sys = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Setup visualization with Irrlicht
vis = irr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Gator on Rigid Terrain')
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.SetCamera(chrono.ChVectorD(0, 2, 15), chrono.ChVectorD(0, 0, 0))
vis.SetTumble(True)

# Create rigid terrain with texture
terrain = veh.RigidTerrain(sys)
patch = terrain.AddPatch(chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0), 200, 200)
patch.SetTexture(chrono.GetChronoDataFile('textures/concrete.jpg'), 200, 200)
sys.Add(terrain.GetGroundBody())
vis.Add(terrain.GetGroundBody())

# Initialize Gator vehicle
vehicle = veh.Gator(sys)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1), chrono.QUNIT))
vehicle.SetChassisFixed(False)
vehicle.SetContactMethod(chrono.ChContactMethod.NTCI)  # Set contact method
vehicle.SetVisualizationType(veh.VisualizationType.MESH)  # Use mesh visualization
vehicle.Initialize()

# Configure TMEasy tires for all wheels
for axle in vehicle.GetAxles():
    for wheel in axle.wheels:
        tire = veh.TMeasyTire(vehicle.GetSystem(), wheel)
        tire.SetType(veh.TMeasyTire.TMeasyType.METRIC)  # Set tire type
        wheel.SetTire(tire)

# Setup driver controls
driver = veh.ChDriver()
vehicle.SetDriver(driver)

# Simulation parameters
step_size = 1.0 / 50.0  # 50 Hz simulation rate

# Main simulation loop
while vis.Run():
    key = vis.GetKeyboard()

    # Update driver inputs based on keyboard
    steering = 0
    throttle = 0
    braking = 0

    if key.IsKeyDown(irr.KEY_KEY_LEFT):
        steering = -1
    elif key.IsKeyDown(irr.KEY_KEY_RIGHT):
        steering = 1

    if key.IsKeyDown(irr.KEY_KEY_UP):
        throttle = 1
    elif key.IsKeyDown(irr.KEY_KEY_DOWN):
        braking = 1

    driver.SetSteering(steering)
    driver.SetThrottle(throttle)
    driver.SetBraking(braking)

    # Update vehicle and system
    vehicle.Synchronize(driver)
    vehicle.Update(step_size)
    sys.DoStepDynamics(step_size)

    # Render scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()