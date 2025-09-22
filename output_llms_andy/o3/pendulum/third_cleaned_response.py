import math
import pychrono.core     as chrono
import pychrono.irrlicht as chronoirr




def make_frame(pos, quat=None):
    if quat is None:
        quat = chrono.ChQuaternionD(1, 0, 0, 0)      
    return chrono.ChFrameD(pos, quat)





sys = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))         





ground = chrono.ChBody()
ground.SetFixed(True)
ground.EnableCollision(False)
sys.AddBody(ground)


ground_pivot = chrono.ChVisualShapeSphere(0.06)
ground_pivot.SetColor(chrono.ChColor(0.4, 0.4, 0.4))
ground.AddVisualShape(ground_pivot)





ROD_RADIUS = 0.05
L1         = 1.0          
L2         = 1.0          
M1         = 1.0
M2         = 1.0

def rod_inertia(mass, length, radius):
    
    J = (1.0 / 12.0) * mass * (3 * radius * radius + length * length)
    return chrono.ChVectorD(J, J, J)





pend1 = chrono.ChBody()
pend1.SetMass(M1)
pend1.SetInertiaXX(rod_inertia(M1, L1, ROD_RADIUS))
pend1.EnableCollision(False)


theta1 = math.radians(45.0)      


p1_cm = chrono.ChVectorD( (L1/2) * math.sin(theta1),
                         -(L1/2) * math.cos(theta1),
                          0)
pend1.SetPos(p1_cm)


vis1 = chrono.ChVisualShapeCylinder(ROD_RADIUS, L1)
vis1.SetColor(chrono.ChColor(0.8, 0.0, 0.0))        
pend1.AddVisualShape(vis1)

sys.AddBody(pend1)


hinge1 = chrono.ChLinkLockRevolute()
hinge1.Initialize(ground, pend1, make_frame(chrono.ChVectorD(0, 0, 0)))
sys.AddLink(hinge1)





pend2 = chrono.ChBody()
pend2.SetMass(M2)
pend2.SetInertiaXX(rod_inertia(M2, L2, ROD_RADIUS))
pend2.EnableCollision(False)

theta2 = math.radians(-30.0)     
theta2_abs = theta1 + theta2     


pivot12 = chrono.ChVectorD( L1 * math.sin(theta1),
                           -L1 * math.cos(theta1),
                            0)


p2_cm = chrono.ChVectorD( pivot12.x + (L2/2) * math.sin(theta2_abs),
                          pivot12.y - (L2/2) * math.cos(theta2_abs),
                          0)
pend2.SetPos(p2_cm)


vis2 = chrono.ChVisualShapeCylinder(ROD_RADIUS, L2)
vis2.SetColor(chrono.ChColor(0.0, 0.0, 0.8))        
pend2.AddVisualShape(vis2)

sys.AddBody(pend2)


hinge2 = chrono.ChLinkLockRevolute()
hinge2.Initialize(pend1, pend2, make_frame(pivot12))
sys.AddLink(hinge2)





vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("PyChrono – Double Pendulum")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(0, 0.4, 3))          





TIME_STEP = 1e-3
logged = False

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    sys.DoStepDynamics(TIME_STEP)

    
    if not logged and sys.GetChTime() > 1.0:
        print("t = %.3f s" % sys.GetChTime())
        print("  pendulum-1 pos =", pend1.GetPos())
        print("  pendulum-2 pos =", pend2.GetPos())
        print("  pendulum-1 vel =", pend1.GetPosDt())
        print("  pendulum-2 vel =", pend2.GetPosDt())
        logged = True