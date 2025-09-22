import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import numpy as np


print('Copyright (c) 2023 Project Chrono')


vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVector3d(-5, -5, 0.5), chrono.ChQuaterniond(1, 0, 0, 0)))
vehicle.SetInitFwdVel(0)
vehicle.SetInitSideVel(0)


vehicle.Initialize()


tire_type = veh.TireModelType_TMEASY
vehicle.SetTireType(tire_type)


vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)


terrain = veh.RigidTerrain(vehicle.GetSystem())
mesh_file = "path/to/Highway_col.obj"  
mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh(mesh_file, True, True)
mesh.Transform(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix33d(1))
terrain.AddMesh(mesh, chrono.ChVector3d(1, 1, 1), chrono.ChVector3d(0, 0, 0))
terrain.Initialize()


vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(vehicle.GetSystem())
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('HMMWV on Custom Mesh Terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(10, 10, 5), chrono.ChVector3d(0, 0, 0))


driver = veh.ChIrrGuiDriver(vis)
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)
driver.Initialize()


while vis.Run():
    time = vehicle.GetSystem().GetChTime()
    driver_inputs = driver.GetInputs()
    vehicle.Advance(time, driver_inputs)
    vis.BeginScene(True, True, irr.SColor(255, 140, 161, 192))
    vis.Render()
    vis.EndScene()
    driver.Synchronize(time)
    vehicle.GetSystem().DoStepDynamics(chrono.ChTimeStep(1 / 50.0))