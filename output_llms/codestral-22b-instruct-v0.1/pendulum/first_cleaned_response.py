import pychrono as chrono
import pychrono.irrlicht as chronoirr

def create_scene():
    
    system = chrono.ChSystemNSC()

    
    ground = chrono.ChBodyEasyBox(10, 1, 1, 1000, True, True)
    system.Add(ground)

    
    pendulum_mass = 1
    pendulum_radius = 0.5
    pendulum_inertia = 2/5 * pendulum_mass * pendulum_radius**2
    pendulum = chrono.ChBodyEasyCylinder(pendulum_radius, pendulum_radius, pendulum_mass, True, True)
    pendulum.SetInertiaXX(chrono.ChVectorD(pendulum_inertia, pendulum_inertia, pendulum_inertia))
    pendulum.SetPos(chrono.ChVectorD(0, -1, 0))
    pendulum.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))

    
    joint = chrono.ChLinkLockRevolute()
    joint.Initialize(ground, pendulum, chrono.ChCoordsysD(chrono.ChVectorD(0, -1, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
    system.Add(joint)

    
    system.Add(pendulum)

    
    vis = chronoirr.ChIrrApp(system, 'Simple Pendulum', chronoirr.dimension2du(800, 600))
    vis.AddTypicalLogo()
    vis.AddTypicalCamera(chronoirr.vector3df(0, 0, -1))
    vis.AddTypicalLights()
    vis.AddSkyBox()
    vis.AddLightWithShadow(chronoirr.vector3df(3, 3, -3), chronoirr.vector3df(0, 0, 0), 9, 1, 7, 40, 512, chronoirr.SColorf(0.8, 0.8, 0.8))

    
    while vis.GetDevice().run():
        vis.BeginScene()
        vis.DrawAll()
        vis.DoStep()

        
        pendulum_position = pendulum.GetPos()
        pendulum_velocity = pendulum.GetPos_dt()
        print(f"Position: {pendulum_position.x}, {pendulum_position.y}, {pendulum_position.z}")
        print(f"Velocity: {pendulum_velocity.x}, {pendulum_velocity.y}, {pendulum_velocity.z}")

        vis.EndScene()


create_scene()