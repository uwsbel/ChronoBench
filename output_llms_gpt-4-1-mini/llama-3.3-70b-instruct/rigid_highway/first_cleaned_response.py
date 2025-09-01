import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


system = chrono.ChSystemNSC()



terrain_collision_mesh = chrono.ChTriangleMesh()
terrain_collision_mesh.LoadWavefrontMesh("Highway_col.obj")


terrain_visual_mesh = chrono.ChTriangleMesh()
terrain_visual_mesh.LoadWavefrontMesh("Highway_vis.obj")


terrain_body = chrono.ChBodyEasyMesh(system, terrain_collision_mesh, 1000, True, True)
terrain_body.SetPos(chrono.ChVectorD(0, -1, 0))
terrain_body.SetBodyFixed(True)


system.Add(terrain_body)


terrain_visual_asset = chrono.ChVisualMesh(terrain_visual_mesh)
terrain_visual_asset.SetWireframe(False)
terrain_body.AddVisualAsset(terrain_visual_asset)



hmmwv = veh.HMMWV()
hmmwv.SetContactMethod(chrono.ChContactMethod.NSC)
hmmwv.SetTireModel(chrono.ChTireModel.TMEASY)
hmmwv.SetChassisPos(chrono.ChVectorD(0, 1, 0))
hmmwv.SetChassisRot(chrono.Q_from_AngX(chrono.CH_C_PI_2))


system.Add(hmmwv.GetChassisBody())
system.Add(hmmwv.GetLeftFrontWheelBody())
system.Add(hmmwv.GetRightFrontWheelBody())
system.Add(hmmwv.GetLeftRearWheelBody())
system.Add(hmmwv.GetRightRearWheelBody())


driver = veh.ChIrrlichtDriver()
driver.SetVehicle(hmmwv)
driver.SetSteeringMin(-0.5)
driver.SetSteeringMax(0.5)
driver.SetThrottleMin(0)
driver.SetThrottleMax(100)
driver.SetBrakingMin(0)
driver.SetBrakingMax(100)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(0, 10, -10))
vis.AddCamera(chrono.ChVectorD(0, 10, 10))
vis.AddTypicalLogo()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("HMMWV Simulation")


vis.Add(hmmwv.GetChassisBody())
vis.Add(hmmwv.GetLeftFrontWheelBody())
vis.Add(hmmwv.GetRightFrontWheelBody())
vis.Add(hmmwv.GetLeftRearWheelBody())
vis.Add(hmmwv.GetRightRearWheelBody())
vis.Add(terrain_body)


time_step = 0.02
vis.SetTimeStep(time_step)


while vis.Run():
    
    system.DoStepDynamics(time_step)
    
    
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()