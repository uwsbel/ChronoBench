import pychrono as chrono
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath(chrono.GetChronoDataPath())


system = chrono.ChSystemNSC()


ground = chrono.ChBody()
ground.SetBodyFixed(True)
system.Add(ground)


pend_length = 2.0  
pend_radius = 0.05 
pend_mass = 1.0    


pend_inertia = chrono.ChVectorD(0.0, (1/3)*pend_mass*pend_length**2, 0.0)

pendulum = chrono.ChBodyEasyCylinder(pend_radius, pend_length, pend_mass, True, True)
pendulum.SetPos(chrono.ChVectorD(0, -pend_length/2, 0))  
pendulum.SetInertiaXX(chrono.ChVectorD(0.5*pend_mass*pend_radius**2, (1/3)*pend_mass*pend_length**2, 0.5*pend_mass*pend_radius**2))
system.Add(pendulum)


revolute_joint = chrono.ChLinkLockRevolute()
revolute_joint.Initialize(pendulum, ground, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0,0,1))))
system.AddLink(revolute_joint)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('PyChrono Pendulum Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(2, 0, 3), chrono.ChVectorD(0, -pend_length/2, 0))
vis.AddTypicalLights()


timestep = 0.01
log_interval = 0.1
next_log_time = 0.0

print("time\tpos_y\tvel_y")
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    time = system.GetChTime()
    if time >= next_log_time:
        pos = pendulum.GetPos()
        vel = pendulum.GetPos_dt()
        print(f"{time:.3f}\t{pos.y:.3f}\t{vel.y:.3f}")
        next_log_time += log_interval

    system.DoStepDynamics(timestep)