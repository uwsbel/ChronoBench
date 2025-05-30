import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math


chrono.SetChronoDataPath('/path/to/chrono/data/')  


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.4)
material.SetRestitution(0.1)


floor = chrono.ChBodyEasyBox(4, 0.2, 2, 1000, True, True, material)
floor.SetPos(chrono.ChVectorD(0, -0.5, 0))
floor.SetBodyFixed(True)
floor.SetName("Floor")
system.Add(floor)


crank_radius = 0.1
crank_length = 0.5
crankshaft = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, crank_radius, crank_length, 7800, True, True, material)
crankshaft.SetPos(chrono.ChVectorD(0, 0.5, 0))
crankshaft.SetName("Crankshaft")
system.Add(crankshaft)


rod_length = 0.8
rod_radius = 0.02
connecting_rod = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, rod_radius, rod_length, 7800, True, True, material)
connecting_rod.SetPos(chrono.ChVectorD(0.3, 0.5, rod_length/2))
connecting_rod.SetName("ConnectingRod")
system.Add(connecting_rod)


piston_size = chrono.ChVectorD(0.15, 0.15, 0.1)
piston = chrono.ChBodyEasyBox(piston_size.x, piston_size.y, piston_size.z, 2700, True, True, material)
piston.SetPos(chrono.ChVectorD(0.3 + rod_length, 0.5, 0))
piston.SetName("Piston")
system.Add(piston)


cylinder = chrono.ChBodyEasyBox(0.3, 0.3, 0.4, 1000, True, True, material)
cylinder.SetPos(chrono.ChVectorD(0.3 + rod_length, 0.5, 0))
cylinder.SetBodyFixed(True)
cylinder.SetName("Cylinder")
system.Add(cylinder)




rev_joint_crank = chrono.ChLinkLockRevolute()
rev_joint_crank.Initialize(floor, crankshaft, 
                          chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), 
                                           chrono.Q_from_AngAxis(chrono.CH_C_PI/2, chrono.VECT_X)))
system.Add(rev_joint_crank)


crank_pin_pos = chrono.ChVectorD(0.3, 0.5, 0)
rev_joint_pin = chrono.ChLinkLockRevolute()
rev_joint_pin.Initialize(crankshaft, connecting_rod,
                        chrono.ChCoordsysD(crank_pin_pos,
                                         chrono.Q_from_AngAxis(chrono.CH_C_PI/2, chrono.VECT_X)))
system.Add(rev_joint_pin)


piston_pin_pos = chrono.ChVectorD(0.3 + rod_length, 0.5, 0)
rev_joint_piston = chrono.ChLinkLockRevolute()
rev_joint_piston.Initialize(connecting_rod, piston,
                           chrono.ChCoordsysD(piston_pin_pos,
                                            chrono.Q_from_AngAxis(chrono.CH_C_PI/2, chrono.VECT_X)))
system.Add(rev_joint_piston)


prismatic_joint = chrono.ChLinkLockPrismatic()
prismatic_joint.Initialize(cylinder, piston,
                          chrono.ChCoordsysD(piston_pin_pos, chrono.QUNIT))
system.Add(prismatic_joint)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(floor, crankshaft,
                chrono.ChFrameD(chrono.ChVectorD(0, 0.5, 0),
                              chrono.Q_from_AngAxis(chrono.CH_C_PI/2, chrono.VECT_X)))

motor_speed = chrono.ChFunction_Const(2.0)
motor.SetSpeedFunction(motor_speed)
system.Add(motor)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Crank-Slider Mechanism Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(2, 1, 2), chrono.ChVectorD(0.5, 0.5, 0))
vis.AddTypicalLights()


light = chronoirr.ChIrrWizard.addLightWithShadow(
    vis.GetDevice(),
    chrono.ChVectorD(10, 10, 10),
    chrono.ChVectorD(0, 0, 0),
    35, 2.2, 50, 512
)



floor.GetVisualShape(0).SetColor(chrono.ChColor(0.6, 0.6, 0.6))
crankshaft.GetVisualShape(0).SetColor(chrono.ChColor(0.8, 0.2, 0.2))
connecting_rod.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.8, 0.2))
piston.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.2, 0.8))
cylinder.GetVisualShape(0).SetColor(chrono.ChColor(0.8, 0.8, 0.2))


system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
system.SetMaxItersSolverSpeed(100)
system.SetMaxItersSolverStab(100)
system.SetTol(1e-6)


time_step = 0.01
simulation_time = 0


def add_simulation_info():
    
    print(f"Simulation Time: {simulation_time:.2f} s")
    print(f"Crankshaft Angular Position: {crankshaft.GetRot().Q_to_Euler123().z:.2f} rad")
    print(f"Piston Position: {piston.GetPos().x:.3f} m")


while vis.Run():
    vis.BeginScene()
    vis.Render()
    
    
    vis.GetGUIEnvironment().getBuiltInFont().draw(
        f"Time: {simulation_time:.2f} s",
        chronoirr.recti(10, 10, 200, 30),
        chronoirr.SColor(255, 255, 255, 255)
    )
    
    crank_angle = crankshaft.GetRot().Q_to_Euler123().z
    vis.GetGUIEnvironment().getBuiltInFont().draw(
        f"Crank Angle: {math.degrees(crank_angle):.1f}°",
        chronoirr.recti(10, 30, 200, 50),
        chronoirr.SColor(255, 255, 255, 255)
    )
    
    piston_pos = piston.GetPos().x
    vis.GetGUIEnvironment().getBuiltInFont().draw(
        f"Piston Position: {piston_pos:.3f} m",
        chronoirr.recti(10, 50, 200, 70),
        chronoirr.SColor(255, 255, 255, 255)
    )
    
    vis.EndScene()
    
    
    system.DoStepDynamics(time_step)
    simulation_time += time_step
    
    
    if int(simulation_time * 100) % 100 == 0:  
        add_simulation_info()


vis.GetDevice().closeDevice()