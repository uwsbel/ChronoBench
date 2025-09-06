import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.fea as fea
import numpy as np


chrono.SetChronoDataPath('/path/to/data')


sys = chrono.ChSystemSMC()
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('HMMWV on SCM Terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()


hmmwv = veh.HMMWV()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(chrono.ChCollisionModel.CollisionType_BULLET)
hmmwv.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI)))
hmmwv.Initialize()


sys.Add(hmmwv)


terrain = veh.SCMterrain()
terrain.SetSoilDensity(1800)
terrain.SetSoilFrictionAngle(30)
terrain.SetSoilCohesion(0)
terrain.SetSoilDilatancy(0)
terrain.SetTerrainLength(100)
terrain.SetTerrainWidth(100)
terrain.SetTerrainHeight(10)


terrain.EnableMovingPatch(True)
patch = terrain.AddPatch(chrono.ChMaterialSurfaceNSC(), chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), terrain.GetTerrainLength(), terrain.GetTerrainWidth())
terrain.Initialize()


sys.Add(terrain)


driver = veh.DriverHMMWV(hmmwv)
driver.SetSteeringInput(0)
driver.SetThrottleInput(0)
driver.SetBrakeInput(0)


while vis.Run():
    
    sys.DoStepDynamics(0.02)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()