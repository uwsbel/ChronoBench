import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.engine as engine
import pychrono.physics as physics


chrono.SetChronoDataPath('path_to_your_chrono_data')


system = chrono.ChSystemNSC()


floor = chrono.ChBody()
floor.SetPos(chrono.ChVectorD(0, -1, 0))
floor.SetMass(1)
floor.SetInertiaXX(chrono.ChVectorD(1, 1, 1))
floor.SetBodyFixed(True)
system.Add(floor)


crankshaft = chrono.ChBody()
crankshaft.SetPos(chrono.ChVectorD(0, 0, 0))
crankshaft.SetMass(1)
crankshaft.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
system.Add(crankshaft)


connecting_rod = chrono.ChBody()
connecting_rod.SetPos(chrono.ChVectorD(0.5, 0, 0))
connecting_rod.SetMass(1)
connecting_rod.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
system.Add(connecting_rod)


piston = chrono.ChBody()
piston.SetPos(chrono.ChVectorD(1, 0, 0))
piston.SetMass(1)
piston.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
system.Add(piston)


crankshaft_joint = chrono.ChLinkRevolute()
crankshaft_joint.Initialize(crankshaft, floor, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 0))
system.Add(crankshaft_joint)


connecting_rod_joint = chrono.ChLinkRevolute()
connecting_rod_joint.Initialize(connecting_rod, crankshaft, chrono.ChVectorD(0.5, 0, 0), chrono.ChVectorD(0, 0, 0))
system.Add(connecting_rod_joint)


piston_joint = chrono.ChLinkPrismatic()
piston_joint.Initialize(piston, connecting_rod, chrono.ChVectorD(1, 0, 0), chrono.ChVectorD(0.5, 0, 0))
system.Add(piston_joint)


motor = chrono.ChFunction_Const()
motor.Set_yconst(chrono.CH_C_PI / 2)  
crankshaft_joint.SetMotorFunction(motor)


visualization = chronoirr.ChVisualSystemIrrlicht()
visualization.AttachSystem(system)
visualization.SetWindowSize(800, 600)
visualization.SetWindowTitle('Crank-Slider Mechanism')
visualization.Initialize()
visualization.AddLogo(chrono.GetChronoDataPath() + 'logo_pychrono_small.png')
visualization.AddTypicalSky()
visualization.AddTypicalLights()


camera = visualization.GetSceneManager().getActiveCamera()
camera.setPosition(chrono.ChVectorD(2, 2, 2))
camera.setTarget(chrono.ChVectorD(0, 0, 0))


run_time = 10  
current_time = 0
time_step = 0.01

while current_time < run_time:
    visualization.Render()
    system.DoStepDynamics(time_step)
    current_time += time_step


visualization.Close()