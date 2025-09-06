import os
import math
import numpy as np
import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


chrono.SetChronoDataPath(os.path.join(os.getcwd(), '..', '..', 'data'))


sys = chrono.ChSystemSMC()
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('HMMWV Simulation')
vis.Initialize()


hmmwv = veh.HMMWV()
hmmwv.SetContactMethod(veh.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.ChCollisionType_BULLET)
hmmwv.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.5), chrono.QUNIT))
hmmwv.Initialize()


terrain = veh.RigidTerrain(hmmwv.GetSystem())
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 100, 100)
terrain.Initialize()


driver = veh.HMMWVDriver()
driver.Initialize(hmmwv)


imu = veh.ChSensorIMU()
gps = veh.ChSensorGPS()
hmmwv.Attach(imu, chrono.ChVector3d(0, 0, 1.5), chrono.ChQuaternionD(1, 0, 0, 0))
hmmwv.Attach(gps, chrono.ChVector3d(0, 0, 1.5), chrono.ChQuaternionD(1, 0, 0, 0))


vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddCamera(chrono.ChVector3d(0, 5, -10))


while vis.Run():
    
    sys.DoStepDynamics(0.01)
    driver.Advance()
    imu.Update()
    gps.Update()

    
    print(f'Vehicle mass: {hmmwv.GetVehicleMass():.2f} kg')

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()


vis.Cleanup()