import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import time


my_system = chrono.ChSystemNSC()
my_system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
my_system.SetMaxItersSolverSpeed(10)


vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(my_system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("HMMWV on Mesh Terrain")
vis.SetCamera(chrono.ChVectorD(0, 5, 20))
vis.AddTypicalLights()
vis.Initialize()


hmmwv = veh.HMMWV(True, True)  
hmmwv.SetContactMethod(chrono.ChContactMethod.NSC)
hmmwv.SetChassisCollisionType(veh.ChassisCollisionType.MESH)
hmmwv.SetTireType(veh.TireModelType.TMEASY)


init_pos = chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1), chrono.QUNIT)
hmmwv.SetInitPosition(init_pos)
hmmwv.Initialize()


material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.9)
terrain = veh.RigidTerrain(my_system)
patch = terrain.AddPatch(material, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0)), 100, 100, False, veh.VisualMaterial.GRASS)
terrain.Initialize()


ground_body = terrain.GetGround()


collision_mesh = chrono.ChTriangleMeshShape()
collision_mesh.SetMesh(chrono.ChTriangleMesh())
collision_mesh.GetMesh().LoadWavefrontMesh(chrono.GetChronoDataFile("Highway_col.obj"))
collision_mesh.Build()


visual_mesh = chrono.ChTriangleMeshShape()
visual_mesh.SetMesh(chrono.ChTriangleMesh())
visual_mesh.GetMesh().LoadWavefrontMesh(chrono.GetChronoDataFile("Highway_vis.obj"))
visual_mesh.Build()


ground_body.GetCollisionModel().ClearModel()
ground_body.GetCollisionModel().AddTriangleMesh(collision_mesh.GetMesh())
ground_body.GetCollisionModel().BuildModel()
ground_body.GetCollisionModel().SetCollide(True)


for i in range(ground_body.GetVisualShapeCount()):
    ground_body.GetVisualShape(i).Remove()
ground_body.AddVisualShape(visual_mesh)


driver = veh.KeyboardDriver()
hmmwv.SetDriver(driver)


time_step = 0.001  
frame_rate = 50
frame_time = 1.0 / frame_rate


start_time = time.time()
while vis.Run():
    
    driver.Synchronize(time_step)
    
    
    my_system.DoStepDynamics(time_step)
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    
    current_time = time.time()
    elapsed = current_time - start_time
    if elapsed < frame_time:
        time.sleep(frame_time - elapsed)
    start_time = current_time


vis.Close()