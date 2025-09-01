import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


print("Copyright (c) 2023 Project Chrono")


veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')
my_veh = veh.ARTcar()
my_veh.SetContactMethod(chrono.ChContactMethod_SMC)
my_veh.SetChassisFixed(False)
my_veh.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(-50, 0, 1.0), chrono.ChQuaternionD(1, 0, 0, 0)))
my_veh.SetPowertrainType(veh.PowertrainModelType_SIMPLE)
my_veh.SetDriveType(veh.DrivelineTypeWV_AWD)
my_veh.SetTireType(veh.TireModelType_TMEASY)
my_veh.SetTireStepSize(1e-3)
my_veh.Initialize()

my_veh.SetChassisVisualizationType(veh.VisualizationType_MESH)
my_veh.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
my_veh.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
my_veh.SetWheelVisualizationType(veh.VisualizationType_MESH)
my_veh.SetTireVisualizationType(veh.VisualizationType_MESH)


terrain = veh.RigidTerrain(my_veh.GetSystem())
patch = terrain.AddPatch(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT), chrono.ChVectorD(200, 200, 0), 0.01)
patch.SetContactFrictionCoefficient(0.9)
patch.SetContactRestitutionCoefficient(0.01)
patch.SetContactMaterialProperties(2e7, 0.3)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
terrain.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('ARTcar Demo')
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 1.75), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AttachVehicle(my_veh.GetVehicle())


driver = veh.ChIrrGuiDriver(vis)
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)
driver.Initialize()


while vis.Run() :
    time = my_veh.GetSystem().GetChTime()
    driver_inputs = driver.GetInputs()
    my_veh.DoStepDynamics(1e-3)
    terrain.Synchronize(time)
    vis.Synchronize(driver.GetInputModeAsString(), driver_inputs)
    vis.Render()

    
    vis.Sleep(1e3/50)