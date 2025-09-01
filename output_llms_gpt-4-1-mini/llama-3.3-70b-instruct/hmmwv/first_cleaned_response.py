import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import numpy as np


sys = chrono.ChSystem()


contact_material = chrono.ChMaterialSurfaceNSC()
contact_material.SetFriction(0.9)
contact_material.SetDampingF(0.5)
contact_material.SetDampingV(0.5)
contact_material.SetCompliance(1e-9)
contact_material.SetComplianceT(1e-9)
sys.SetDefaultContactMaterial(contact_material)


hmmwv = veh.HMMWV()
hmmwv.SetContactMethod(chrono.ChContactMethod.NSC)
hmmwv.SetTireType(veh.TireModelType.TMEASY)
hmmwv.SetChassisFixed(False)
hmmwv.SetSuspensionType(veh.SuspensionType.SPRING_DAMPER)
hmmwv.SetVehiclePos(chrono.ChVectorD(0, 0.5, 0))
hmmwv.SetVehicleRot(chrono.Q_from_AngX(chrono.CH_C_PI_2))


sys.Add(hmmwv.GetChassisBody())
sys.Add(hmmwv.GetPart(0))
sys.Add(hmmwv.GetPart(1))
sys.Add(hmmwv.GetPart(2))
sys.Add(hmmwv.GetPart(3))
sys.Add(hmmwv.GetPart(4))
sys.Add(hmmwv.GetPart(5))


terrain = chrono.ChBodyEasyBox(sys, 100, 10, 100, 1000, True, True)
terrain.SetPos(chrono.ChVectorD(0, -10, 0))
terrain.SetMaterial(contact_material)
sys.Add(terrain)


texture = chronoirr.ChTexture()
texture.SetTextureFile("terrain_texture.jpg")
terrain.AddAsset(texture)


hmmwv.GetChassisBody().AddAsset(chrono.ChBoxShape(hmmwv.GetChassisBody(), 1.5, 1.5, 3.5))
hmmwv.GetPart(0).AddAsset(chrono.ChBoxShape(hmmwv.GetPart(0), 1, 1, 2))
hmmwv.GetPart(1).AddAsset(chrono.ChBoxShape(hmmwv.GetPart(1), 1, 1, 2))
hmmwv.GetPart(2).AddAsset(chrono.ChBoxShape(hmmwv.GetPart(2), 1, 1, 2))
hmmwv.GetPart(3).AddAsset(chrono.ChBoxShape(hmmwv.GetPart(3), 1, 1, 2))
hmmwv.GetPart(4).AddAsset(chrono.ChBoxShape(hmmwv.GetPart(4), 1, 1, 2))
hmmwv.GetPart(5).AddAsset(chrono.ChBoxShape(hmmwv.GetPart(5), 1, 1, 2))


driver = veh.ChIrrVehicleDriver(hmmwv)


driver.SetSteeringGain(0.1)
driver.SetThrottleGain(0.1)
driver.SetBrakingGain(0.1)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.AddSkyBox()
vis.AddTypicalLights()


time_step = 0.01
duration = 10


while sys.GetChTime() < duration:
    
    sys.DoStepDynamics(time_step)
    
    
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()
    
    
    driver.Synchronize(time_step)
    
    
    chrono.ChSleep(1 / 50)