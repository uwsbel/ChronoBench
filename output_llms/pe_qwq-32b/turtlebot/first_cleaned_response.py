import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


sys = chrono.ChSystemSMC()
sys.SetGravity(chrono.ChVectorD(0, 0, -9.8))  


vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Turtlebot Simulation')
vis.AttachSystem(sys)
vis.SetCamera(chronoirr.ChVectorD(0, 5, 5), chronoirr.ChVectorD(0, 0, 0), chronoirr.ChVectorD(0, 1, 0))
vis.AddTypicalLights()
vis.Initialize()


terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChMaterialSurfaceNSC()
terrain.AddPatch(patch_mat, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0)), 100, 100)
terrain.Initialize()



chassis = chrono.ChBodyEasyBox(0.5, 0.1, 0.2, 1000, True, True)
chassis.SetPos(chrono.ChVectorD(0, 0, 0.1))  
sys.Add(chassis)


left_wheel = chrono.ChBodyEasyCylinder(0.05, 0.2, 1000, True, True)
left_wheel.SetPos(chrono.ChVectorD(-0.25, 0, 0.1))
sys.Add(left_wheel)


right_wheel = chrono.ChBodyEasyCylinder(0.05, 0.2, 1000, True, True)
right_wheel.SetPos(chrono.ChVectorD(0.25, 0, 0.1))
sys.Add(right_wheel)


rev_left = chrono.ChLinkLockRevolute()
rev_left.Initialize(chassis, left_wheel, chrono.ChFrameD(chrono.ChVectorD(-0.25, 0, 0)))
sys.Add(rev_left)

rev_right = chrono.ChLinkLockRevolute()
rev_right.Initialize(chassis, right_wheel, chrono.ChFrameD(chrono.ChVectorD(0.25, 0, 0)))
sys.Add(rev_right)


motor_left = chrono.ChLinkMotorRotationSpeed()
motor_left.Initialize(chassis, left_wheel, rev_left.GetFrame())
sys.Add(motor_left)

motor_right = chrono.ChLinkMotorRotationSpeed()
motor_right.Initialize(chassis, right_wheel, rev_right.GetFrame())
sys.Add(motor_right)


chassis.AddVisualShape(chrono.ChVisualShapeBox(0.5, 0.1, 0.2))
left_wheel.AddVisualShape(chrono.ChVisualShapeCylinder(0.05, 0.2))
right_wheel.AddVisualShape(chrono.ChVisualShapeCylinder(0.05, 0.2))


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    current_time = sys.GetChTime()
    
    if 2 <= current_time < 3:
        
        motor_left.SetSpeedFunction(chrono.ChFunction_Const(-2.0))
        motor_right.SetSpeedFunction(chrono.ChFunction_Const(2.0))
    elif 4 <= current_time < 5:
        
        motor_left.SetSpeedFunction(chrono.ChFunction_Const(2.0))
        motor_right.SetSpeedFunction(chrono.ChFunction_Const(-2.0))
    else:
        
        motor_left.SetSpeedFunction(chrono.ChFunction_Const(0.0))
        motor_right.SetSpeedFunction(chrono.ChFunction_Const(0.0))
    
    sys.DoStepDynamics(0.01)