import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

# Create the simulation system and add a contact material
sys = chrono.ChSystemNSC()
contact_material = chrono.ChMaterialSurfaceNSC()
contact_material.SetFriction(0.9)
contact_material.SetDampingF(0.000)
contact_material.SetCompliance(0.0001)
contact_material.SetComplianceT(0.0001)
sys.SetContactMaterial(contact_material)

# Create the vehicle and set its parameters
veh_hmmwv = veh.HMMWV()
veh_hmmwv.SetContactMaterial(contact_material)
veh_hmmwv.SetChassisCollisionModel(chrono.ChBox(1.2, 0.8, 1.8))
veh_hmmwv.SetChassisVisualization(chrono.ChVisualizationBOX())
veh_hmmwv.SetChassisMass(1500)
veh_hmmwv.SetChassisInertiaXX(chrono.ChVectorD(1000, 1000, 1000))
veh_hmmwv.SetChassisPosition(chrono.ChVectorD(0, 0.8, 0))
veh_hmmwv.SetChassisOrientation(chrono.ChQuaternionD(chrono.ChVectorD(0, 1, 0), chrono.ChAngleD(chrono.CH_C_PI_2)))
veh_hmmwv.SetTireType(veh.TireModelType.TMEASY)
veh_hmmwv.SetTireStepSize(0.01)
veh_hmmwv.SetTireForceEnabled(True)
veh_hmmwv.SetSteeringType(veh.SteeringType.STEERING_SINGLE_TRACK)
veh_hmmwv.SetBrakeType(veh.BrakeType.BRAKE_SIMPLE)
veh_hmmwv.SetDrivetrainType(veh.DrivetrainType.DRIVETRAIN_TWO_WHEELS_DIFFERENTIAL)

# Create the terrain and add it to the system
terrain = chrono.ChBodyEasyMesh()
terrain.SetMesh(chrono.ChMesh(chrono.GetChronoDataFile("Highway_col.obj")))
terrain.SetCollide(True)
terrain.SetBodyFixed(True)
terrain.SetPos(chrono.ChVectorD(0, -0.5, 0))
terrain.SetMaterial(contact_material)
sys.Add(terrain)

# Create the visual terrain and add it to the system
terrain_vis = chrono.ChBodyEasyMesh()
terrain_vis.SetMesh(chrono.ChMesh(chrono.GetChronoDataFile("Highway_vis.obj")))
terrain_vis.SetCollide(False)
terrain_vis.SetBodyFixed(True)
terrain_vis.SetPos(chrono.ChVectorD(0, -0.5, 0))
sys.Add(terrain_vis)

# Create the driver system and add it to the vehicle
driver = veh.ChIrrGuiDriver()
driver.SetSteeringMode(veh.SteeringMode.STEERING_MODE_ANGLE)
driver.SetThrottleMode(veh.ThrottleMode.THROTTLE_MODE_PERCENT)
driver.SetBrakingMode(veh.BrakingMode.BRAKING_MODE_PERCENT)
driver.SetVehicle(veh_hmmwv)
veh_hmmwv.AddDriver(driver)

# Create the Irrlicht application and add the vehicle and terrain to the scene
app = chronoirr.ChVisualSystemIrrlicht()
app.SetWindowSize(800, 600)
app.SetWindowTitle("HMMWV Simulation")
app.AddCamera(chrono.ChVectorD(0, 1.5, -3), chrono.ChVectorD(0, 1, 0))
app.AddLightWithShadow(chrono.ChVectorD(1, 2, 2), chrono.ChVectorD(0, 0, 0), 10, 10, 10, 100, 100, 100)
app.AddSkyBox()
app.AddTypicalLigthing()
app.AddVehicle(veh_hmmwv)
app.AddTerrain(terrain_vis)

# Run the simulation loop
while app.Run():
    app.BeginScene()
    app.DrawAll()
    sys.DoStepDynamics(0.02)
    app.EndScene()
    chrono.ChUtils::Sleep(0.02)