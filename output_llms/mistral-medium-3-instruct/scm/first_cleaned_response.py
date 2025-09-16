import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import numpy as np


chrono.SetChronoDataPath("PATH_TO_CHRONO_DATA")  
vehicle_dir = "vehicle/data/hmmwv"


system = chrono.ChronoSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))


hmmwv = veh.HMMWV()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoorD(0, 0, 0.5))
hmmwv.SetInitFwdVel(5)
hmmwv.Initialize()


hmmwv.SetTireType(veh.RigidTire)
hmmwv.SetTireStepSize(0.01)


hmmwv.GetSystem().Add(hmmwv.GetChassis())
hmmwv.GetSystem().Add(hmmwv.GetPowertrain())
hmmwv.GetSystem().Add(hmmwv.GetSteering())

for axle in hmmwv.GetAxles():
    system.Add(axle)
    for wheel in axle.GetWheels():
        system.Add(wheel)


terrain = veh.SCMDeformableTerrain(hmmwv.GetSystem())
terrain.SetPlane(chrono.ChCoorD(0, 0, 0, chrono.QUNIT))
terrain.SetLength(20)
terrain.SetWidth(10)
terrain.Initialize()


terrain.SetSoilParameters(
    Bekker_Kphi=1e6,
    Bekker_Kc=1e5,
    Bekker_n=1.0,
    Mohr_cohesion=1e3,
    Mohr_friction=35,
    Mohr_dilation=5,
    Janosi_shear=0.01,
    plastic_yield=0.01
)


terrain.EnableMovingPatch(True, 10, 5)
terrain.SetMovingPatchReference(hmmwv.GetChassis())


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("HMMWV on SCM Terrain")
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(5, 0, 2), chrono.ChVectorD(0, 0, 0.5))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(10, 10, 10), chrono.ChVectorD(0, 0, 0), 15, 1, 15, 40, 512)


terrain.GetMesh().SetFalseColorRange(0, 0.2)
terrain.GetMesh().SetFalseColorMode(chronoirr.FalseColorMode::Z_DISPLACEMENT)


veh.ChVehicleVisualSystemIrrlicht::AddVisualModel(hmmwv, vis)


driver = veh.ChDriverHMMWV(vis)
driver.Initialize()


time_step = 0.02  
while vis.Run():
    time = system.GetChTime()
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)