import pychrono as chrono
import pychrono.irrlicht as irr
import random


chrono.SetChronoDataPath('PYCHRONO_DATA_DIR')
my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


my_application = irr.ChIrrApp(my_system, 'Particle Attraction Simulation', irr.dimension2du(1024, 768))
my_application.AddTypicalLogo()
my_application.AddTypicalSky()
my_application.AddTypicalCamera(irr.vector3df(0, 3, -6))
my_application.AddTypicalLights()


emitter_position = chrono.ChVectorD(0, 3, 0)
emitter_velocity = chrono.ChVectorD(0, 0, 0)


def create_particle():
    
    shape_choice = random.choice([chrono.ChBodyEasyBox, chrono.ChBodyEasySphere, chrono.ChBodyEasyCylinder])
    particle = shape_choice(1, 1, 1, 1000)

    
    particle.SetPos(emitter_position + chrono.ChVectorD(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1)))
    particle.SetPos_dt(emitter_velocity + chrono.ChVectorD(random.uniform(-0.1, 0.1), random.uniform(-0.1, 0.1), random.uniform(-0.1, 0.1)))
    particle.SetRot(chrono.ChQuaternionD(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1)).GetNormalized())

    
    my_system.Add(particle)



def custom_gravity(body, time, force):
    force += body.GetMass() * 0.5 * chrono.ChVectorD(0, -9.81, 0)


my_system.AddForceField(custom_gravity)


while my_application.GetDevice().run():
    my_application.BeginScene()
    my_application.DrawAll()
    my_application.DoStep()

    
    create_particle()

    my_application.EndScene()