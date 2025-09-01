import pychrono as chrono
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath('/path/to/chrono/data/')


system = chrono.ChSystemNSC()


ground = chrono.ChBody()
ground.SetBodyFixed(True)
system.Add(ground)


pendulum = chrono.ChBody()
pendulum.SetPos(chrono.ChVectorD(0, -1, 0))
pendulum.SetMass(1.0)
pendulum.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))


pendulum_shape = chrono.ChBoxShape()
pendulum_shape.GetBoxGeometry().Size = chrono.ChVectorD(0.05, 0.5, 0.05)
pendulum.AddAsset(pendulum_shape)


system.Add(pendulum)


joint = chrono.ChLinkLockRevolute()
joint.Initialize(ground, pendulum, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
system.Add(joint)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Simple Pendulum Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 0, -3))
vis.AddTypicalLights()


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    
    pos = pendulum.GetPos()
    vel = pendulum.GetPos_dt()
    print(f'Time: {system.GetChTime():.2f} s, Position: {pos}, Velocity: {vel}')
    
    
    system.DoStepDynamics(0.01)