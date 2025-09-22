import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr




sys = chrono.ChSystemNSC()


sys.Set_G_acc(chrono.ChVectorD(0, -1.62, 0))




ground = chrono.ChBody()
ground.SetBodyFixed(True)
sys.AddBody(ground)

pivot_pos   = chrono.ChVectorD(0, 0, 1)   
pivot_radius = 2.0                        

pivot_shape = chrono.ChVisualShapeSphere(pivot_radius)
pivot_shape.SetColor(chrono.ChColor(0.3, 0.3, 0.3))
ground.AddVisualShape(pivot_shape, chrono.ChFrameD(pivot_pos))




rod_len     = 1.5
rod_radius  = 0.1

pend = chrono.ChBody()
pend.SetBodyFixed(False)
pend.SetMass(2.0)
pend.SetInertiaXX(chrono.ChVectorD(0.4, 1.5, 1.5))


pend.SetPos(chrono.ChVectorD(0.5 * rod_len, 0, 1))


pend.SetWvel_par(chrono.ChVectorD(0, 0, 2.0))


rod_shape = chrono.ChVisualShapeCylinder(rod_radius, rod_len)
rod_shape.SetColor(chrono.ChColor(0.6, 0, 0))
rod_orientation = chrono.Q_from_AngAxis(math.pi / 2.0, chrono.ChVectorD(0, 0, 1))
pend.AddVisualShape(rod_shape, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), rod_orientation))

sys.AddBody(pend)




sph_joint = chrono.ChLinkLockSpherical()
sph_joint.Initialize(ground, pend,
                     chrono.ChCoordsysD(pivot_pos, chrono.QUNIT))
sys.AddLink(sph_joint)




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Single pendulum – spherical joint (Moon gravity)')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 3, 6))
vis.AddTypicalLights()




step_size = 1e-3
logged = False

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    sys.DoStepDynamics(step_size)

    
    if not logged and sys.GetChTime() > 1.0:
        com   = pend.GetPos()
        v_com = pend.GetPos_dt()
        print(f"\nTime = {sys.GetChTime():.3f} s")
        print(f"    COM position  :  x = {com.x:.3f}  y = {com.y:.3f}  z = {com.z:.3f}")
        print(f"    COM velocity  :  x = {v_com.x:.3f}  y = {v_com.y:.3f}  z = {v_com.z:.3f}")
        logged = True