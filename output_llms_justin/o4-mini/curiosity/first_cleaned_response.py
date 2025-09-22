import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh




chrono.SetChronoDataPath("C:/Chrono/data/")     
chrono.SetChronoThreads(4)
sys = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))




ground = chrono.ChBody()
ground.SetBodyFixed(True)

ground_collision = chrono.ChBoxShape()
ground_collision.GetBoxGeometry().Size = chrono.ChVectorD(10, 0.1, 10)
ground.AddAsset(ground_collision)
ground.GetCollisionModel().ClearModel()
ground.GetCollisionModel().AddBox(0.1, 10, 0.1, chrono.ChVectorD(0,0,0))
ground.GetCollisionModel().BuildModel()
ground.SetCollide(True)

vis_mat = chrono.ChVisualMaterial()
vis_mat.SetKdTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
ground.AddAsset(chrono.ChTriangleMeshShape())
ground.SetVisualShape(0, chrono.ChFrameD(chrono.ChVectorD(0,0,0)))
ground.AddAsset(vis_mat)

sys.Add(ground)





rover = veh.Curiosity_Rover()

initPos = chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.Q_from_AngY(0))
rover.Initialize(sys, initPos)

rover.SetChassisCollision(True)
rover.SetChassisVisualizationType(veh.VisualizationType.MESH)
rover.SetWheelVisualizationType(veh.VisualizationType.MESH)






throttle_fn = chrono.ChFunction_Const(0.3)

steer_fn = chrono.ChFunction_Sine(0.1, 0.5, 0.0)

brake_fn = chrono.ChFunction_Const(0.0)

driver = veh.ChDriver(rover)
driver.SetThrottleFunction(throttle_fn)
driver.SetSteeringFunction(steer_fn)
driver.SetBrakingFunction(brake_fn)
driver.Initialize()




app = chronoirr.ChIrrApp(
    sys,
    "PyChrono: Curiosity Rover on Rigid Terrain",
    chronoirr.dimension2du(1280, 720)
)
app.AddTypicalLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
app.AddSkyBox()
app.AddTypicalLights()
app.AddShadowAll()

cam_pos = chrono.ChVectorD(-4, 2, -4)
cam_target = chrono.ChVectorD(0, 0.5, 0)
app.AddCamera(cam_pos, cam_target)


app.AssetBindAll()
app.AssetUpdateAll()




time_step = 1.0 / 1000  
while app.Run():
    
    driver.GetInputs()
    
    app.BeginScene()
    app.DrawAll()
    app.EndScene()
    
    sys.DoStepDynamics(time_step)