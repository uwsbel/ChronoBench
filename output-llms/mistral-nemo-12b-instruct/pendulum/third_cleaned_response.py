import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math


sys = chrono.ChSystemNSC()


sys.Set_Gravitational_Acceleration(chrono.ChVector3d(0, -9.81, 0))  


ground = chrono.ChBody()
sys.Add(ground)
ground.SetFixed(True)  
ground.Enable_Collision(False)  


cyl_ground = chrono.ChVisualShapeCylinder(0.2, 0.4)  
ground.Add_Visual_Shape(cyl_ground, chrono.ChFramed(chrono.ChVector3d(0, 0, +1)))


pend_1 = chrono.ChBody()
sys.Add_Body(pend_1)
pend_1.SetFixed(False)  
pend_1.Enable_Collision(False)  
pend_1.Set_Mass(1)  
pend_1.Set_Inertia_XX(chrono.ChVector3d(0.2, 1, 1))  


cyl_pend_1 = chrono.ChVisualShapeCylinder(0.2, 2)  
cyl_pend_1.Set_Color(chrono.ChColor(0.6, 0, 0))  
pend_1.Add_Visual_Shape(cyl_pend_1, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))


pend_1.Set_Pos(chrono.ChVector3d(1, 0, 1))


rev_1 = chrono.ChLinkLockRevolute()
rev_1.Initialize(ground, pend_1, chrono.ChFramed(chrono.ChVector3d(0, 0, 1), chrono.ChQuaterniond(1, 0, 0, 0)))
sys.Add_Link(rev_1)


pend_2 = chrono.ChBody()
sys.Add_Body(pend_2)
pend_2.SetFixed(False)  
pend_2.Enable_Collision(False)  
pend_2.Set_Mass(1)  
pend_2.Set_Inertia_XX(chrono.ChVector3d(0.2, 1, 1))  


cyl_pend_2 = chrono.ChVisualShapeCylinder(0.2, 2)  
cyl_pend_2.Set_Color(chrono.ChColor(0, 0, 0.6))  
pend_2.Add_Visual_Shape(cyl_pend_2, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))


pend_2.Set_Pos(chrono.ChVector3d(2, 0, 1))


rev_2 = chrono.ChLinkLockRevolute()
rev_2.Initialize(pend_1, pend_2, chrono.ChFramed(chrono.ChVector3d(0, 0, 1), chrono.ChQuaterniond(1, 0, 0, 0)))
sys.Add_Link(rev_2)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.Attach_System(sys)  
vis.Set_Window_Size(1024, 768)  
vis.Set_Window_Title('Double Pendulum demo')  
vis.Initialize()  
vis.Add_Logo(chrono.Get_Chrono_Data_File('logo_pychrono_alpha.png'))  
vis.Add_SkyBox()  
vis.Add_Camera(chrono.ChVector3d(0, 3, 6))  
vis.Add_Typical_Lights()  


log_info = True  
while vis.Run():
    vis.Begin_Scene()  
    vis.Render()  
    vis.End_Scene()  
    sys.Do_Step_Dynamics(1e-3)  

    
    if log_info and sys.Get_Ch_Time() > 1:
        pos_1 = pend_1.Get_Pos()  
        pos_2 = pend_2.Get_Pos()  
        print("t = ", sys.Get_Ch_Time())
        print("     ", pos_1.x, "  ", pos_1.y)
        print("     ", pos_2.x, "  ", pos_2.y)
        log_info = False