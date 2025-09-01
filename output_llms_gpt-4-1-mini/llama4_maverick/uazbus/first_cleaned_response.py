import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math


print('Copyright (c) 2023 Project Chrono')


veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')
uazbus_vehicle = veh.UAZBUS(veh.SteeringType_POWERTRAIN, False, veh.BrakeType_SIMPLE)


initLoc = chrono.ChVectorD(0, 0, 1.0)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)
uazbus_vehicle.Initialize(chrono.ChCoordsysD(initLoc, initRot))


terrain = veh.RigidTerrain(uazbus_vehicle.GetSystem())
patch = terrain.AddPatch(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT), 
                         chrono.ChVectorD(100, 100, 0), 0.0)
patch.SetContactFrictionCoefficient(0.9)
patch.SetContactRestitutionCoefficient(0.01)
patch.SetContactMaterialProperties(2e7, 0.3)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
terrain.Initialize()


driver = veh.ChDriver(uazbus_vehicle.GetVehicle())
driver.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('UAZBUS Demo')
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 1.75), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(uazbus_vehicle.GetVehicle())


while vis.Run():
    time = uazbus_vehicle.GetSystem().GetChTime()
    driver_inputs = driver.GetInputs()
    uazbus_vehicle.Advance(time)
    terrain.Advance(time)
    driver.Advance(time)
    vis.Advance(time)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    irr.ChIrrAppUtilities.sleep(1)