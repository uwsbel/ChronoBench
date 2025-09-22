import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

# Initialize the PyChrono environment
print("Copyright (c) 2023 Project PyChrono")

# Create the FEDA vehicle
veh.FEDA_initJSON = "FEDA/JSON/FEDA_Vehicle.json"
my_feda = veh.FEDAVehicle(veh.FEDA_initJSON)

# Set the contact method
my_feda.SetContactMethod(chrono.ChContactMethod_SMC)

# Initialize the vehicle at a specified position and orientation
initLoc = chrono.ChVectorD(-5, 0, 0.7)
initRot = chrono.Q_from_AngY(chrono.CH_C_PI / 2)
my_feda.Initialize(chrono.ChCoordsysD(initLoc, initRot))

# Set tire model
my_feda.SetTireType(veh.TireModelType_TMEASY)

# Get the vehicle and set visualization type for all vehicle parts
feda_vehicle = my_feda.GetVehicle()
feda_vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
feda_vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
feda_vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
feda_vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)

# Create the terrain
terrain = veh.RigidTerrain(my_feda.GetSystem())
patch = terrain.AddPatch(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, -5), chrono.Q_from_AngX(-chrono.CH_C_PI / 2)), chrono.ChVectorD(20, 20), 0.01)
patch.SetContactFrictionCoefficient(0.9)
patch.SetContactRestitutionCoefficient(0.01)
patch.SetContactMaterial(0.9, 0.01, 2e7)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetVisualizationType(veh.VisualizationType_MESH)

# Initialize the Irrlicht visualization
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('FEDA Vehicle Simulation')
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 1.75), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AttachVehicle(feda_vehicle)

# Create the interactive driver system
driver = veh.ChIrrGuiDriver(vis)
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)
driver.Initialize()

# Simulation loop
while vis.Run() :
    time = my_feda.GetSystem().GetChTime()
    driver_inputs = driver.GetInputs()
    my_feda.Advance(time - my_feda.GetSystem().GetChTime())
    vis.Advance(driver_inputs)
    vis.Render()
    my_feda.GetSystem().DoStepDynamics(0.02)