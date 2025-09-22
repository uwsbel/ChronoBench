import chrono
from chrono import ChVector, ChQuaternion, ChFrame, ChCoordsys
from chrono.irrlicht import ChirrApp
from chrono.collision import ChCollisionModel
from chrono.irrlicht import ChIrrDevice
from chrono.irrlicht.ChIrrTools import idummy
from chrono.vehicle import ChVehicleModel, ChSteering, ChWheel, ChTire, ChSimpleFWDTire
from chrono.utils import ChFunctions


chrono.SetChronoDataPath('path/to/chrono/data')
chrono.SetChronoDevice(ChIrrDevice())



ground = chrono.ChBodyEasyBox(1, 0.1, 1, 100, True, True)
system.AddBody(ground)


pendulum_length = 1  
pendulum_mass = 1  
pendulum_radius = 0.1  

pendulum_inertia = chrono.ChVectorD(pendulum_mass * pendulum_radius ** 2, pendulum_mass * pendulum_radius ** 2, pendulum_mass * pendulum_radius ** 2)
pendulum = chrono.ChBodyEasySphere(pendulum_radius, pendulum_mass, pendulum_inertia)


pendulum_pos = ChVectorD(0, 1.5 * pendulum_length, 0)
pendulum.SetPos(pendulum_pos)
pendulum.SetPos_dt(ChVectorD(0, 0, 0))


joint = chrono.ChLinkLockRevolute()
joint.Initialize(ground, pendulum, chrono.ChCoordsysD(pendulum_pos))
system.AddLink(joint)


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


vis = ChirrApp(system.GetDevice(), 'Simple Pendulum Simulation', chrono.ChVector2i(800, 600))
vis.AddCamera(chrono.ChVectorD(0, 5, 5), chrono.ChVectorD(0, 0, 0))
vis.AddLogo(chrono.GetChronoDataPath() + '/logo_pychrono_white.png')
vis.AddSkyBox()
vis.AddTypicalLights()


ground_shape = chrono.ChBoxShape()
ground_shape.BoxExtents = chrono.ChVectorD(0.5, 0.05, 0.5)
ground_shape.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
ground.AddVisualShape(ground_shape)

pendulum_shape = chrono.ChSphereShape()
pendulum_shape.Radius = pendulum_radius
pendulum_shape.SetColor(chrono.ChColor(0.5, 0.1, 0.1))
pendulum.AddVisualShape(pendulum_shape)


def my_log_function(sim, sph):
    pos = sph.GetPos()
    vel = sph.GetPos_dt()
    print(f'Time: {sim.GetChTime()} Pendulum position: {pos} Velocity: {vel}')

pendulum.AddPostStepCallBack(my_log_function)


vis.Run()