import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import matplotlib.pyplot as plt
import numpy as np




sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0)) 


crank_center = chrono.ChVector3d(-1, 0.5, 0)
crank_rad = 0.4
crank_thick = 0.1 
rod_length = 1.5




mfloor = chrono.ChBodyEasyBox(4, 1, 4, 1000) 
mfloor.SetPos(chrono.ChVector3d(0, -0.5 + crank_center.y, 0)) 
mfloor.SetFixed(True)
sys.Add(mfloor)


mcrank = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, crank_rad, crank_thick, 1000)
mcrank.SetPos(crank_center) 
mcrank.SetRot(chrono.Q_ROTATE_Y_TO_Z) 
sys.Add(mcrank)


mrod = chrono.ChBodyEasyBox(rod_length, 0.1, 0.1, 1000)
rod_initial_pos_x = crank_center.x + crank_rad + rod_length / 2
mrod.SetPos(chrono.ChVector3d(rod_initial_pos_x, crank_center.y, crank_center.z))
sys.Add(mrod)


mpiston = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.2, 0.3, 1000) 
piston_initial_pos = chrono.ChVector3d(crank_center.x + crank_rad + rod_length, crank_center.y, crank_center.z)
mpiston.SetPos(piston_initial_pos)
mpiston.SetRot(chrono.Q_ROTATE_Y_TO_X) 
sys.Add(mpiston)




my_motor = chrono.ChLinkMotorRotationSpeed()
my_motor.Initialize(mcrank,
                    mfloor,
                    chrono.ChFramed(crank_center, chrono.QUNIT)) 
my_angularspeed = chrono.ChFunctionConst(chrono.CH_PI)  
my_motor.SetMotorFunction(my_angularspeed)
sys.Add(my_motor)


mjointA = chrono.ChLinkLockSpherical()
crank_pin_abs_pos = chrono.ChVector3d(crank_center.x + crank_rad, crank_center.y, crank_center.z)
mjointA.Initialize(mrod,
                   mcrank,
                   chrono.ChCoordsysd(crank_pin_abs_pos)) 
sys.Add(mjointA)


mjointB = chrono.ChLinkLockSpherical()

piston_pin_abs_pos = piston_initial_pos
mjointB.Initialize(mpiston,
                   mrod,
                   chrono.ChCoordsysd(piston_pin_abs_pos)) 
sys.Add(mjointB)


mjointC = chrono.ChLinkLockPlanar()


planar_joint_frame_abs = chrono.ChFramed(piston_initial_pos, chrono.QUNIT)
mjointC.Initialize(mpiston,
                   mfloor,
                   planar_joint_frame_abs)
sys.Add(mjointC)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Crank demo - Modified Joints')
vis.Initialize()
try:
    vis.AddLogo(chrono.GetChrono
print("error happened with only start ```python")