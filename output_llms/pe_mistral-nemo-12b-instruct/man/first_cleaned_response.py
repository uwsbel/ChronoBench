import os
import math
import numpy as np
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import pychrono.robot as robosimian


chrono.SetChronoDataPath(os.path.join(os.getcwd(), 'chrono_data'))


sys = chrono.ChSystemSMC()
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('MAN 10t Truck Simulation')
vis.Initialize()


car = veh.MAN10t()
car.SetContactMethod(veh.ChContactMethod_NSC)
car.SetChassisCollisionType(veh.ChCollisionType_BULLET)
car.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
car.Initialize()


car.SetTireModel(veh.TireModel_TMEASY)


terrain = veh.RigidTerrain(car.GetSystem())
patch_mat = veh.RigidTerrainPatchMaterial()
patch_mat.SetTexture(chrono.GetChronoDataFile('grass.jpg'))
patch_mat.SetNormalMap(chrono.GetChronoDataFile('grass_normal.png'))
patch_mat.SetSpecularMap(chrono.GetChronoDataFile('grass_specular.png'))
patch_mat.SetLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
patch_mat.SetLogoSize(0.2)
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 100, 100)
terrain.Initialize()


driver = veh.ChVehicleDriver()
driver.SetSteeringFunction(chrono.ChFunction_Const(0.0))
driver.SetThrottleFunction(chrono.ChFunction_Const(0.0))
driver.SetBrakeFunction(chrono.ChFunction_Const(0.0))
car.SetDriver(driver)


vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 5, -10), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()


while vis.Run():
    
    if vis.IsKeyPressed('LEFT'):
        driver.SetSteeringFunction(chrono.ChFunction_Const(-1.0))
    elif vis.IsKeyPressed('RIGHT'):
        driver.SetSteeringFunction(chrono.ChFunction_Const(1.0))
    else:
        driver.SetSteeringFunction(chrono.ChFunction_Const(0.0))

    if vis.IsKeyPressed('UP'):
        driver.SetThrottleFunction(chrono.ChFunction_Const(1.0))
    else:
        driver.SetThrottleFunction(chrono.ChFunction_Const(0.0))

    if vis.IsKeyPressed('DOWN'):
        driver.SetBrakeFunction(chrono.ChFunction_Const(1.0))
    else:
        driver.SetBrakeFunction(chrono.ChFunction_Const(0.0))

    
    sys.DoStepDynamics(0.01)

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

vis.GetDevice().waitForAllEvents()
vis.GetDevice().closeDevice()