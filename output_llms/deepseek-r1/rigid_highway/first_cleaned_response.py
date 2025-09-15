import os
import math
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


chrono.SetChronoDataPath(os.path.join(os.getenv('CHRONO_DATA_DIR'), 'data/'))
system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))


vehicle = veh.HMMWV_Full(system)
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT))
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.SetTireStepSize(0.001)
vehicle.Initialize()


vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)


ground = chrono.ChBody()
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground.SetBodyFixed(True)


collision_mesh = chrono.ChTriangleMeshConnected()
collision_mesh.LoadWavefrontMesh(chrono.GetChronoDataFile('Highway_col.obj'))
ground.GetCollisionModel().ClearModel()
ground.GetCollisionModel().AddTriangleMesh(collision_mesh, True, False, chrono.ChVectorD(0,0,0))
ground.GetCollisionModel().BuildModel()
ground.SetCollide(True)


visual_mesh = chrono.ChObjFileShape()
visual_mesh.SetFilename(chrono.GetChronoDataFile('Highway_vis.obj'))
ground.AddVisualShape(visual_mesh)
system.Add(ground)


vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('HMMWV on Custom Terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 10, 2))
vis.AddTypicalLights()


driver = veh.ChInteractiveDriverIRR(vis)
steering_controller = veh.ChSteeringControllerSMC()
driver.SetSteeringController(steering_controller)
driver.SetInputDataFile(veh.GetDataFile("MDF/driver/Accelerations.txt"))
driver.SetInputMode(veh.InputMode_DATAFILE)
driver.Initialize()


time_step = 0.002
real_time_frame = 0.02  
time = 0

while vis.Run():
    time = system.GetChTime()
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    
    driver_inputs = driver.GetInputs()
    vehicle.GetDriver().SetInputs(driver_inputs)
    
    
    driver.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    system.DoStepDynamics(time_step)
    
    
    chrono.ChRealtimeStep(real_time_frame)