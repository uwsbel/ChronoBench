import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math




CRANK_RADIUS    = 0.2       
CRANK_WIDTH     = 0.05      
CON_ROD_LENGTH  = 0.6       
PISTON_RADIUS   = 0.07      
PISTON_HEIGHT   = 0.12      
MOTOR_SPEED     = 2.0       




system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
system.SetSolverMaxIterations(150)




mat_floor    = chrono.ChMaterialSurfaceNSC()
mat_parts    = chrono.ChMaterialSurfaceNSC()




floor = chrono.ChBodyEasyBox(3.0, 0.1, 1.0, 1000, True, True, mat_floor)
floor.SetPos(chrono.ChVectorD(0.5, -0.15, 0))
floor.SetBodyFixed(True)
floor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(floor)


pillar = chrono.ChBodyEasyBox(0.08, 0.3, 0.08, 1000, True, True, mat_floor)
pillar.SetPos(chrono.ChVectorD(0.0, 0.1, 0.0))
pillar.SetBodyFixed(True)
pillar.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
system.Add(pillar)


guide = chrono.ChBodyEasyBox(CON_ROD_LENGTH + CRANK_RADIUS + 0.15, 
                              PISTON_RADIUS * 2.2,
                              PISTON_RADIUS * 2.2, 
                              1000, True, True, mat_floor)
guide.SetPos(chrono.ChVectorD(0.5 * (CON_ROD_LENGTH + CRANK_RADIUS + 0.15) + CRANK_RADIUS * 0.5,
                               0.0, 0.0))
guide.SetBodyFixed(True)
guide.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/chrome.png"))
system.Add(guide)







crank = chrono.ChBodyEasyBox(CRANK_RADIUS, CRANK_WIDTH, CRANK_WIDTH,
                              2700, True, True, mat_parts)



crank.SetPos(chrono.ChVectorD(CRANK_RADIUS / 2.0, 0.0, 0.0))
crank.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/redwhite.png"))
system.Add(crank)









con_rod_cx = CRANK_RADIUS + CON_ROD_LENGTH / 2.0

con_rod = chrono.ChBodyEasyBox(CON_ROD_LENGTH, 0.04, 0.04,
                                2700, True, True, mat_parts)
con_rod.SetPos(chrono.ChVectorD(con_rod_cx, 0.0, 0.0))
con_rod.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/greenwhite.png"))
system.Add(con_rod)




piston_x = CRANK_RADIUS + CON_ROD_LENGTH   

piston = chrono.ChBodyEasyCylinder(PISTON_RADIUS, PISTON_HEIGHT,
                                    2700, True, True, mat_parts)

piston.SetPos(chrono.ChVectorD(piston_x, 0.0, 0.0))
rot90z = chrono.ChQuaternionD()
rot90z.Q_from_AngZ(math.pi / 2.0)
piston.SetRot(rot90z)
piston.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))
system.Add(piston)







motor = chrono.ChLinkMotorRotationSpeed()
motor_frame = chrono.ChFrameD(chrono.ChVectorD(0, 0, 0),
                               chrono.ChQuaternionD(1, 0, 0, 0))
motor.Initialize(crank, floor, motor_frame)
motor_speed_func = chrono.ChFunction_Const(MOTOR_SPEED)
motor.SetSpeedFunction(motor_speed_func)
system.Add(motor)



rev_crank_rod = chrono.ChLinkLockRevolute()
rev_crank_rod.Initialize(
    crank, con_rod,
    chrono.ChCoordsysD(chrono.ChVectorD(CRANK_RADIUS, 0, 0),
                        chrono.ChQuaternionD(1, 0, 0, 0))
)
system.Add(rev_crank_rod)



rev_rod_piston = chrono.ChLinkLockRevolute()
rev_rod_piston.Initialize(
    con_rod, piston,
    chrono.ChCoordsysD(chrono.ChVectorD(CRANK_RADIUS + CON_ROD_LENGTH, 0, 0),
                        chrono.ChQuaternionD(1, 0, 0, 0))
)
system.Add(rev_rod_piston)




prismatic = chrono.ChLinkLockPrismatic()
rot_prismatic = chrono.ChQuaternionD()
rot_prismatic.Q_from_AngY(math.pi / 2.0)          
prismatic.Initialize(
    piston, floor,
    chrono.ChCoordsysD(chrono.ChVectorD(piston_x, 0, 0), rot_prismatic)
)
system.Add(prismatic)




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("PyChrono – Crank-Slider Mechanism")
vis.Initialize()


vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()


vis.AddLight(chrono.ChVectorD( 4,  6,  3), 8.0, chrono.ChColor(0.9, 0.9, 1.0))
vis.AddLight(chrono.ChVectorD(-4,  4, -3), 6.0, chrono.ChColor(1.0, 0.9, 0.8))


vis.AddCamera(chrono.ChVectorD(0.5, 0.8, 2.0),   
              chrono.ChVectorD(0.5, 0.0, 0.0))    


vis.AddTypicalLights()




time_step = 1e-3    
end_time  = 20.0    

realtime_timer = chrono.ChRealtimeStepTimer()

print("=" * 60)
print("  Crank-Slider Simulation")
print(f"  Crank radius   : {CRANK_RADIUS} m")
print(f"  Connecting rod : {CON_ROD_LENGTH} m")
print(f"  Motor speed    : {MOTOR_SPEED} rad/s  "
      f"({MOTOR_SPEED/(2*math.pi)*60:.1f} RPM)")
print("=" * 60)

while vis.Run() and system.GetChTime() < end_time:
    vis.BeginScene(True, True, chrono.ChColor(0.18, 0.25, 0.35))
    vis.DrawAll()

    
    vis.GetGUIEnvironment()   

    vis.EndScene()

    system.DoStepDynamics(time_step)
    realtime_timer.Spin(time_step)

    
    t = system.GetChTime()
    if abs(t % 0.5) < time_step * 1.5:
        px = piston.GetPos().x
        print(f"  t = {t:6.2f} s  |  piston x = {px:.4f} m")

print("\nSimulation complete.")