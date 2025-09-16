import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as chronovehicle







my_system = chrono.ChSystemNSC()
my_system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
my_system.SetSolverMaxIterations(100)
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))







terrain_heightmap = chrono.ChHeightmap(
    "terrain_heightmap.png",  
    100,  
    100,  
    5,  
)

terrain_material = chrono.ChMaterialSurfaceNSC()
terrain_material.SetFriction(0.8)
terrain_material.SetRestitution(0.2)

terrain = chrono.ChTerrain(terrain_heightmap, terrain_material)
my_system.Add(terrain)







vehicle = chronovehicle.ChVehicle("HMMWV", my_system)
vehicle.SetChassisFixed(False)
vehicle.SetChassisPosition(chrono.ChVectorD(0, 1, 0))


vehicle.SetVehicleModel(chronovehicle.ChVehicleModelHMMWV())
vehicle.Initialize()







for i in range(vehicle.GetNumWheels()):
    wheel = vehicle.GetWheel(i)
    wheel.SetVisualizationType(chronovehicle.ChWheelVisualizationType_MESH)
    wheel.SetTireModel(chronovehicle.ChTireModelRigid())







scm_patch = chrono.ChSCMDeformableTerrainPatch(terrain, 10, 10)
scm_patch.SetMaterial(chrono.ChMaterialSurfaceNSC())
scm_patch.SetSinkageFactor(0.5)
scm_patch.SetStiffness(1000)
scm_patch.SetDamping(100)
scm_patch.SetMovingPatch(True)


scm_patch.SetTargetBody(vehicle.GetChassis())







vis = chronoirr.ChIrrApp(my_system, "HMMWV Simulation", chronoirr.dimension2du(1280, 720))
vis.AddTypicalLights()
vis.AddSkyBox()
vis.SetCameraPosition(chrono.ChVectorD(0, 5, 5))
vis.SetCameraLookAt(chrono.ChVectorD(0, 1, 0))


vehicle.SetChassisVisualizationType(chronovehicle.ChChassisVisualizationType_MESH)
for i in range(vehicle.GetNumWheels()):
    vehicle.GetWheel(i).SetVisualizationType(chronovehicle.ChWheelVisualizationType_MESH)


scm_patch.SetVisualizationType(chrono.ChVisualizationType_SCM_DEFORMABLE_TERRAIN)
scm_patch.SetSinkageVisualization(True)







driver = chronovehicle.ChInteractiveDriver(vehicle)
vis.SetDriver(driver)







while vis.GetDevice().run():
    vis.BeginScene()
    vis.Render()
    my_system.DoStepDynamics(0.02)
    vis.EndScene()