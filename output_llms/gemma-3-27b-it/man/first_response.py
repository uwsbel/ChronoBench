import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

# --- Create the simulation system ---
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Set gravity

# --- Create the terrain ---
terrain = chrono.ChTerrain()
terrain.Create(system, "terrain", chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(100, 2, 100), 20, 20)
terrain.SetTexture("textures/terrain.png", "textures/terrain_normal.png")
terrain.SetLogo("textures/chrono_logo.png")
system.Add(terrain)

# --- Create the vehicle ---
vehicle = veh.ChTruck(system)
vehicle.SetVehicleModel("MAN_10t")  # Choose a vehicle model
vehicle.SetTireModel(veh.ChTireEasy)
vehicle.SetEngineType(veh.ChEngineSimple)
vehicle.SetTransmissionType(veh.ChTransmissionSimple)
vehicle.SetDifferentialType(veh.ChDifferentialSimple)

# --- Vehicle setup ---
vehicle.SetInitPosition(chrono.ChVectorD(0, 1, 0))
vehicle.SetInitRotation(chrono.Q_from_AngAxis(chrono.CH_C_PI / 2, chrono.ChVectorD(0, 0, 1))) # Rotate 90 degrees around Z
vehicle.SetChassisFixed(False)
vehicle.SetTireForceRadius(0.3)
vehicle.SetTireForceOffset(0.0)

# --- Visualization settings ---
vehicle.SetCollidable(True)
vehicle.SetVisualizationType(veh.VehicleVisualizationType.MESH)
vehicle.SetMeshFile("models/man_10t/truck.obj")
vehicle.SetMaterial("truck_mat", True)

# --- Driver controls ---
driver = veh.ChDriver()
driver.SetSteeringDelta(10.0)
driver.SetThrottleDelta(0.1)
driver.SetBrakingDelta(0.2)
vehicle.SetDriver(driver)

# --- Real-time controls ---
def keyboard_event(key, pressed):
    if key == chrono.KEY_UP:
        driver.SetThrottle(1.0 if pressed else 0.0)
    elif key == chrono.KEY_DOWN:
        driver.SetBraking(1.0 if pressed else 0.0)
    elif key == chrono.KEY_LEFT:
        driver.SetSteering(0.5 if pressed else 0.0)
    elif key == chrono.KEY_RIGHT:
        driver.SetSteering(-0.5 if pressed else 0.0)

# --- Create the Irrlicht application ---
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('MAN 10t Truck Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 5, -10))
vis.AddTypicalLights()
vis.AddSkyBox()
vis.SetChaseCamera(True)

# --- Simulation loop ---
system.SetTimestepperType(chrono.ChTimestepper.Type_EulerExplicit)
system.SetStepTime(0.01)

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    system.DoStepDynamics()

    # Handle keyboard input
    for i in range(vis.GetNumKeys()):
        if vis.IsKeyPressed(i):
            keyboard_event(i, True)
        else:
            keyboard_event(i, False)