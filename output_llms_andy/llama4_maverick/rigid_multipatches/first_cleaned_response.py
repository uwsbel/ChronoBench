import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import numpy as np






initLoc = chrono.ChVectorD(0, 0, 1.0)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)


chassis_vis_type = veh.VisualizationType_MESH
wheel_vis_type = veh.VisualizationType_MESH
tire_vis_type = veh.VisualizationType_MESH


step_size = 2e-3






vehicle = veh.HMMWV_Full(
    initLoc, 
    initRot, 
    veh.ChassisCollisionType_NONE
)


vehicle.SetChassisVisualizationType(chassis_vis_type)
vehicle.SetWheelVisualizationType(wheel_vis_type)
vehicle.SetTireVisualizationType(tire_vis_type)


vehicle.SetEngineType(veh.EngineModelType_SIMPLE)
vehicle.SetDriveType(veh.DrivetrainModelType_SIMPLE)


vehicle.Initialize()






terrain = veh.RigidTerrain(vehicle.GetSystem())


patch1_mat = chrono.ChMaterialSurfaceSMC()
patch1 = terrain.AddPatch(patch1_mat, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(10, 0, 0), chrono.ChVectorD(0, 10, 0))
patch1.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 10, 10)
patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.5))


patch2_mat = chrono.ChMaterialSurfaceSMC()
patch2 = terrain.AddPatch(patch2_mat, chrono.ChVectorD(12, 0, 0), chrono.ChVectorD(22, 0, 0), chrono.ChVectorD(12, 10, 0))
patch2.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 10, 10)
patch2.SetColor(chrono.ChColor(0.5, 0.8, 0.5))


mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh(veh.GetDataFile("terrain/meshes/bump.obj"))
patch3_mat = chrono.ChMaterialSurfaceSMC()
patch3 = terrain.AddPatch(patch3_mat, mesh, chrono.ChVectorD(5, 5, 0))
patch3.SetColor(chrono.ChColor(0.8, 0.5, 0.5))


br = chrono.ChVectorD(8, 8, 1)
mesh = chrono.ChGridMesh(br, 50, 50)
hmat = chrono.ChMatrixDynamic(1, 1)
for i in range(mesh.GetN().x):
    for j in range(mesh.GetN().y):
        hmat(i, j) = 0.1 * np.sin(3 * i / mesh.GetN().x) * np.cos(3 * j / mesh.GetN().y)
mesh.ResetElevation(hmat)
patch4_mat = chrono.ChMaterialSurfaceSMC()
patch4 = terrain.AddPatch(patch4_mat, mesh, chrono.ChVectorD(-4, -4, 0))
patch4.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 2, 2)


terrain.Initialize()






vis = irr.ChVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV on Complex Terrain')
vis.SetWindowSize(1280, 720)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, -10, 2), chrono.ChVectorD(0, 0, 0))
vis.AttachVehicle(vehicle)






driver = veh.ChDriver(vehicle)


while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    
    driver_inputs = driver.GetInputs()
    driver_inputs.SetSteering(0.0)
    driver_inputs.SetThrottle(0.5)
    driver_inputs.SetBraking(0.0)

    
    driver.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    terrain.Synchronize(time)
    vis.Synchronize("", terrain)
    vis.Synchronize("", vehicle)

    
    vehicle.GetSystem().DoStepDynamics(step_size)
    vis.Advance(step_size)

    
    vis.Render()