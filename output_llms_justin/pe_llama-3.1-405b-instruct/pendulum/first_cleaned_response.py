import pychrono as chrono
import pychrono.irrlicht as chronoirr


sys = chrono.ChSystemNSC()


material = chrono.ChContactMaterialNSC()
material.SetFriction(0.5)
material.SetDampingF(0.1)
material.SetCompliance(0.01)
sys.SetContactMethod(chrono.ChContactMethod_NSC)


ground = chrono.ChBody()
ground.SetBodyFixed(True)
sys.Add(ground)


pendulum = chrono.ChBody()
pendulum.SetMass(1.0)
pendulum.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))
pendulum.SetPos(chrono.ChVector3d(0, -1, 0))
sys.Add(pendulum)


rev_joint = chrono.ChLinkLockRevolute()
rev_joint.Initialize(pendulum, ground, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
sys.Add(rev_joint)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Pendulum Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 6))
vis.AddTypicalLights()


while vis.Run():
    
    sys.DoStepDynamics(0.01)

    
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()

    
    print(f'Pendulum position: {pendulum.GetPos()}')
    print(f'Pendulum velocity: {pendulum.GetPos_dt()}')