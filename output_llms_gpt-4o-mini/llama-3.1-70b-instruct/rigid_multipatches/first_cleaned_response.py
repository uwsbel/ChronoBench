import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import numpy as np


chrono.SetChronoDataPath('./data/')


mysystem = chrono.ChSystemNSC()
mysystem.SetSolverType(chrono.ChSolver.Type_PSSOR)
mysystem.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


veh_params = veh.HMMWV()
veh_params.SetContactMethod(chrono.ChContactMethodType.MULTICORE_SMC)
veh_params.SetChassisFixed(False)
veh_params.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.Q_from_AngX(chrono.CH_C_PI_2)))
veh_params.SetPowertrainType(veh.PowertrainType_SHAFTS)
veh_params.SetDriveType(veh.DrivetrainType_FOUR_WHEEL_DRIVE)
veh_params.SetEngineType(veh.EngineType_INTERNAL_COMBUSTION)
veh_params.SetBrakeType(veh.BrakeType_SHAFTS)
veh_params.SetSteeringType(veh.SteeringTypeAckermann())
veh_params.SetTireType(veh.TireModelType_FrictionCircle)
veh_params.SetTireStepSize(chrono.ChTime(0.01))


my_vehicle = veh.HMMWV(veh_params)
mysystem.Add(my_vehicle.GetChassisBody())
mysystem.Add(my_vehicle.GetChassisBodyAuxRef())


for axle in my_vehicle.GetAxles():
    mysystem.Add(axle)
for wheel in my_vehicle.GetWheels():
    mysystem.Add(wheel)
for tire in my_vehicle.GetTires():
    mysystem.Add(tire)


terrain = chrono.ChBodyEasyBox(mysystem, 100, 10, 100, 1000, True, True)
terrain.SetPos(chrono.ChVectorD(0, -10, 0))
terrain.SetBodyFixed(True)
mysystem.Add(terrain)


patch1 = chrono.ChBodyEasyBox(mysystem, 20, 2, 20, 1000, True, True)
patch1.SetPos(chrono.ChVectorD(-20, -8, 0))
patch1.SetBodyFixed(True)
mysystem.Add(patch1)


patch2 = chrono.ChBodyEasyBox(mysystem, 20, 2, 20, 1000, True, True)
patch2.SetPos(chrono.ChVectorD(20, -8, 0))
patch2.SetBodyFixed(True)
mysystem.Add(patch2)


bump = chrono.ChBodyEasyMesh(mysystem, 'bump.obj', 1000, True, True)
bump.SetPos(chrono.ChVectorD(0, -8, 20))
bump.SetBodyFixed(True)
mysystem.Add(bump)


heightmap = chrono.ChBodyEasyHeightField(mysystem, 'heightmap.png', 1000, True, True)
heightmap.SetPos(chrono.ChVectorD(0, -8, -20))
heightmap.SetBodyFixed(True)
mysystem.Add(heightmap)


driver = veh.ChIrrGuiDriver(my_vehicle)
driver.SetSteeringMode(veh.SteeringMode_TYPE_2)


application = chronoirr.ChVisualSystemIrrlicht()
application.SetWindowSize(1280, 720)
application.AddTypicalLights()
application.AddSkyBox()
application.AddLogo('logo.png')
application.SetShowLogo(True)
application.SetCamera(chrono.ChVectorD(0, 1.5, -3), chrono.ChVectorD(0, 1, 0))
application.SetCameraUp(chrono.ChVectorD(0, 1, 0))
application.SetCameraSpeed(0.5)
application.SetCameraRotSpeed(0.1)


for axle in my_vehicle.GetAxles():
    application.AssetBindAllChildren(axle)
for wheel in my_vehicle.GetWheels():
    application.AssetBindAllChildren(wheel)
for tire in my_vehicle.GetTires():
    application.AssetBindAllChildren(tire)


application.AssetBindAllChildren(terrain)
application.AssetBindAllChildren(patch1)
application.AssetBindAllChildren(patch2)
application.AssetBindAllChildren(bump)
application.AssetBindAllChildren(heightmap)


mysystem.SetTimestep(0.01)
application.SetTimestep(0.01)
application.SetTryRealtime(True)


while application.GetDevice().run():
    mysystem.DoStepDynamics(0.01)
    application.BeginScene()
    application.DrawAll()
    application.EndScene()