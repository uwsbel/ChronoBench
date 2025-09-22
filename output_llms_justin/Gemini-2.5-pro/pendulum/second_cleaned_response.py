import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


















sys = chrono.ChSystemNSC()


sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -1.62, 0))  



ground = chrono.ChBody()
sys.Add(ground)
ground.SetFixed(True)  
ground.EnableCollision(False)  



ground_cyl_vis = chrono.ChVisualShapeCylinder(0.2, 0.4)  
ground.AddVisualShape(ground_cyl_vis, chrono.ChFramed(chrono.ChVector3d(0, 0, 1))) 


pend_1 = chrono.ChBody()
sys.AddBody(pend_1)
pend_1.SetFixed(False)  
pend_1.EnableCollision(False)  


pend_1.SetMass(2.0)  
pend_1.SetInertiaXX(chrono.ChVector3d(0.4, 1.5, 1.5))  




pend_cyl_radius = 0.1  
pend_cyl_height = 1.5 
L_pend = pend_cyl_height / 2.0 

pend_cyl_vis = chrono.ChVisualShapeCylinder(pend_cyl_radius, pend_cyl_height)
pend_cyl_vis.SetColor(chrono.ChColor(0.6, 0, 0))  






pend_1.AddVisualShape(pend_cyl_vis, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleZ(chrono.CH_PI_2)))



pivot_point_abs = chrono.ChVector3d(0, 0, 1)



pend_1.SetPos(pivot_point_abs + chrono.ChVector3d(L_pend, 0, 0))

pend_1.SetRot(chrono.QUNIT)




pend_1.SetAngVelParent(chrono.ChVector3d(0, 2.0, 0.5))



sph_joint = chrono.ChLinkLockSpherical()




sph_joint.Initialize(ground, pend_1, chrono.ChFramed(pivot_point_abs))
sys.AddLink(sph_joint)



joint_vis_sphere = chrono.ChVisualShapeSphere(2.0) 
joint_vis_sphere.SetColor(chrono.ChColor(0.2, 0.2, 0.8))  
sph_joint.AddVisualShape(joint_vis_sphere)



vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Modified Pendulum: Spherical Joint, Moon Gravity, Initial Velocity') 
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()


camera_pos_relative = chrono.ChVector3d(L_pend * 3, L_pend * 2.5, L_pend * 2)
vis.AddCamera(pivot_point_abs + camera_pos_relative, pivot_point_abs)
vis.AddTypicalLights()


time_step = 1e-3  
log_info = True  

while vis.Run():
    vis.BeginScene()  
    vis.Render()      
    vis.EndScene()    

    sys.DoStepDynamics(time_step)  

    
    if log_info and sys.GetChTime() >= 1.0:
        pos_1 = pend_1.GetPos()  
        lin_vel_1 = pend_1.GetPosDt()  
        ang_vel_1_local = pend_1.GetAngVelLocal() 

        print(f"--- Simulation info at t = {sys.GetChTime():.3f} s ---")
        print(f"Pendulum CoM Position:      x={pos_1.x:.3f}, y={pos_1.y:.3f}, z={pos_1.z:.3f} m")
        print(f"Pendulum CoM Linear Vel.:   x={lin_vel_1.x:.3f}, y={lin_vel_1.y:.3f}, z={lin_vel_1.z:.3f} m/s")
        print(f"Pendulum Ang. Vel. (local): x={ang_vel_1_local.x:.3f}, y={ang_vel_1_local.y:.3f}, z={ang_vel_1_local.z:.3f} rad/s")
        
        log_info = False