import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import os




TIME_STEP = 0.001
SIMULATION_TIME = 30.0
TERRAIN_SIZE = 10.0  
TERRAIN_HEIGHT = 0.2  
ROVER_INIT_POS = chrono.ChVectorD(0, 1.5, 0)
CAMERA_POS = chrono.ChVectorD(3, 2, 3)  




system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  
system.SetSolverMaxIterations(100)
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)




ground = chrono.ChBodyEasyBox(TERRAIN_SIZE, TERRAIN_HEIGHT, TERRAIN_SIZE, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -TERRAIN_HEIGHT/2, 0))
ground.SetBodyFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)




rover = veh.Curiosity(system)
rover.Initialize(chrono.ChFrameD(ROVER_INIT_POS))
rover.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)


motor_speed = 0.3  
steering_speed = 0.05  
driver = rover.GetDriver()
driver.SetDesiredSpeed(motor_speed)
driver.SetSteering(steering_speed)




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Curiosity Rover Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(CAMERA_POS, ROVER_INIT_POS)
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(5, 8, -5), ROVER_INIT_POS, 15, 2, 10, 40, 512)


vis.EnableShadows()




time = 0.0
while vis.Run() and time < SIMULATION_TIME:
    vis.BeginScene()
    vis.Render()
    vis.RenderFrame(ROVER_INIT_POS, chrono.QUNIT, 1.0)  
    vis.EndScene()
    
    
    driver.Update(chrono.ChTimer())
    
    
    system.DoStepDynamics(TIME_STEP)
    time += TIME_STEP

    
    cam_pos = rover.GetChassis().GetPos() + CAMERA_POS
    vis.GetActiveCamera().setTarget(chronoirr.vector3df(
        rover.GetChassis().GetPos().x,
        rover.GetChassis().GetPos().y,
        rover.GetChassis().GetPos().z))