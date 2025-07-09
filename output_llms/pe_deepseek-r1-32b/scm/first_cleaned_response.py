import math
import numpy as np
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.vehicle.deformable as vehdef
import pychrono.robot as rob
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath(chrono.GetChronoDataFile(''))


sys = chrono.ChSystemSMC()
sys.SetSolverType(chrono.ChSolver.Type_BARZILAI_BORWEIN)
sys.SetSolverMaxIterations(100)
sys.SetSolverTolerance(1e-6)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV on Deformable Terrain')
vis.SetWindowSize(1280, 720)
vis.SetNumberOfCameras(1)
vis.SetCameraPosition(chrono.ChVector3d(0, 5, 20))
vis.SetCameraRotation(chrono.ChVector3d(0, -60, 0))
vis.EnableLight(0, True)
vis.EnableLight(1, True)
vis.EnableLight(2, True)
vis.EnableLight(3, True)
vis.EnableSkyBox()
vis.EnableGrid()
vis.AttachSystem(sys)


hmmwv = veh.HMMWV()
hmmwv.SetContactMethod(chrono.ChContactMethod.NSC)
hmmwv.SetChassisCollisionType(veh.ChassisCollisionType.BOX)
hmmwv.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
hmmwv.SetTireType(veh.TireType.RIGID)
hmmwv.SetTireRadius(0.48)
hmmwv.SetTireWidth(0.25)
hmmwv.SetTireFriction(0.8)
hmmwv.Initialize()


terrain = vehdef.RigidTerrain(sys)
terrain.SetMaterial(vehdef.DeformableMaterial.SAND)
terrain.SetShearModulus(1e6)
terrain.SetDamping(0.1)
terrain.SetYoungModulus(1e7)
terrain.SetPoissonRatio(0.3)
terrain.SetDensity(1.5)
terrain.SetThickness(0.2)
terrain.SetSize(20, 20)
terrain.SetPosition(chrono.ChVector3d(0, 0, 0))
terrain.SetMovingPatch(True)
terrain.SetMovingPatchOffset(chrono.ChVector3d(0, 0, 0))
terrain.SetMovingPatchVelocity(chrono.ChVector3d(0, 0, 0))
terrain.Initialize()


hmmwv.Chassis().AddVisualShape(chrono.ChVisualShapeBox(chrono.ChVector3d(3.8, 2.0, 0.8), chrono.ChColor(0.5, 0.5, 0.5)))
for axle in hmmwv.GetAxles():
    axle.GetAxleTube().AddVisualShape(chrono.ChVisualShapeCylinder(0.1, 1.0, chrono.ChColor(0.3, 0.3, 0.3)))
for wheel in hmmwv.GetWheels():
    wheel.AddVisualShape(chrono.ChVisualShapeCylinder(0.12, 0.24, chrono.ChColor(0.8, 0.8, 0.8)))


terrain.GetTerrainMesh().AddVisualShape(chrono.ChVisualShapeMesh(chrono.ChTriangleMesh(), chrono.ChColor(0.2, 0.2, 0.2)))


driver = veh.VehicleDriverInputs()
driver.SetThrottle(0.5)
driver.SetSteering(0)
driver.SetBraking(0)


simulation_time = 0
render_time = 0
frame_rate = 50
time_step = 1.0 / frame_rate


while vis.Run():
    
    driver.Update()
    
    
    hmmwv.Update(time_step, driver)
    
    
    terrain.Update(time_step, hmmwv.GetChassis().GetPos())
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    
    simulation_time += time_step
    render_time += time_step
    
    
    if render_time >= 1.0 / frame_rate:
        render_time = 0
        vis.DoStepDynamics(time_step)