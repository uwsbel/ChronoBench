import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


sys = chrono.ChSystemSMC()
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBody()
ground.SetBodyFixed(True)
sys.Add(ground)


robot = veh.Turtlebot()
robot.SetContactMethod(chrono.ChContactMethod_NSC)
robot.SetChassisCollisionType(chrono.ChCollisionSystem.Type_BULLET)
robot.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.1, 0), chrono.QUNIT))
robot.Initialize()
sys.Add(robot)


terrain = veh.RigidTerrain(sys)
patch = terrain.AddPatch(chrono.ChMaterialSurfaceNSC(), chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT), 10, 10)
terrain.Initialize()


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Turtlebot Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 2, 4))
vis.AddTypicalLights()


time_step = 0.01
time = 0
left_motor = robot.GetMotor(0)
right_motor = robot.GetMotor(1)

while vis.Run():
    
    if time < 5:
        left_motor.SetMotorFunction(chrono.ChFunction_Const(0.5))
        right_motor.SetMotorFunction(chrono.ChFunction_Const(-0.5))  
    elif time < 10:
        left_motor.SetMotorFunction(chrono.ChFunction_Const(-0.5))
        right_motor.SetMotorFunction(chrono.ChFunction_Const(0.5))  
    else:
        left_motor.SetMotorFunction(chrono.ChFunction_Const(0))
        right_motor.SetMotorFunction(chrono.ChFunction_Const(0))  

    
    sys.DoStepDynamics(time_step)
    time += time_step

    
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()