import pychrono as chrono
import pychrono.irrlicht as chronoirr


system = chrono.ChSystemNSC()


time_step = 1e-3
out_step = 1e-2


hlength = 1.0  
hh = 2 * hlength  
hw = chrono.ChVector3d(0, hlength, 0)  
hd = chrono.ChVector3d(0, -hlength, 0)  


crankrad = 0.4  
cranklen = 2.2  
motor_angle = chrono.ChQuaterniond(1, 0, 0, 0)  


rod_w = 0.4  
rod_h = 3.0  
rod_W = 1.0  
crank2 = 1.5  
piston_rad = 0.6  
piston_len = 0.4  


center_ground = chrono.ChVector3d(0, -1.1, 0)  
crank_center = chrono.ChVector3d(0, -1.1, 0)  
rod_center = chrono.ChVector3d(0, 1, 0)  
piston_center = chrono.ChVector3d(0, 2, 0)  


truss_floor = chrono.ChBodyEasyBox(10, 1, 10, 1000)  
truss_floor.SetPos(chrono.ChVector3d(0, -7, 0))  
truss_floor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))  
system.Add(truss_floor)  


crank = chrono.ChBodyEasyCylinder(crankrad, cranklen, 1000)  
crank.SetPos(crank_center + hw)  
crank.SetRot(chrono.Q_FROM_AXIS_ANGLE_Y, chrono.CH_PI_2)  
crank.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))  
system.Add(crank)  


rod_casing = chrono.ChBodyEasyBox(rod_w, rod_h, rod_W, 1000)  
rod_casing.SetPos(rod_center + hw)  
rod_casing.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/brick.jpg"))  
system.Add(rod_casing)  


piston = chrono.ChBodyEasyCylinder(piston_rad, piston_len, 1000)  
piston.SetPos(piston_center + hw)  
piston.SetRot(chrono.Q_FROM_AXIS_ANGLE_Y, chrono.CH_PI_2)  
piston.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/pink.png"))  
system.Add(piston)  


motor = chrono.ChLinkMotorRotationSpeed()  
motor_frames = chrono.ChFramesys(
    (crank_center + hd + hw).GetX(),
    (crank_center + hd + hw).GetY(),
    (crank_center + hd + hw).GetZ(),
    1, 0, 0, 0)  
motor.Initialize(crank, truss_floor, motor_frames)  
speed_fun = chrono.ChFunctionConst(chrono.CH_PI)  
motor.SetSpeedFunction(speed_fun)  
system.Add(motor)  


crank_bearing = chrono.ChLinkLockRevolute()  
crank_bearing.Initialize(crank, truss_floor, chrono.ChFramed(
    crank_center + hw))  
system.AddLink(crank_bearing)  


rod_crank_bearing = chrono.ChLinkLockRevolute()  
rod_crank_bearing.Initialize(
    rod_casing, crank, chrono.ChFramed(crank_center + chrono.VNULL + hw))  
system.AddLink(rod_crank_bearing)  


rod_piston_bearing = chrono.ChLinkLockRevolute()  
rod_piston_bearing.Initialize(
    rod_casing, piston, chrono.ChFramed(piston_center + chrono.VNULL + hw))  
system.AddLink(rod_piston_bearing)  


truss_piston_bearing = chrono.ChLinkLockPrismatic()  
truss_piston_bearing.Initialize(
    piston, truss_floor, chrono.ChFramed(piston_center + hw, chrono.Q_FROM_AXIS_ANGLE_X, 90 * chrono.CH_DEG_TO_RAD))  
system.AddLink(truss_piston_bearing)  


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)  
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)  
vis.SetWindowSize(1024, 768)  
vis.SetWindowTitle('ChLinkMotorRotationSpeed demo')  
vis.Initialize()  
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  
vis.AddSkyBox()  
vis.AddCamera(chrono.ChVector3d(3, 13, 8), chrono.ChVector3d(0, 3, 0))  
vis.AddTypicalLights()  


time = 0
while vis.Run():
    time += time_step  

    
    if time > out_time:
        time_str = f'{time:0.3f}'  
        print(f'             time = {time_str} s')
        out_time += out_step  

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)