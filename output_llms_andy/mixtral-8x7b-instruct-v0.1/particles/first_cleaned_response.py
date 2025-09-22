import chrono
from chrono import ChSystem, ChBody, ChVector, ChQuaternion, ChCoGeomFriction, ChWreckingBall, ChLinkMotorRotation, ChLinkEngine
from chrono.irrlicht import ChirrApp
from random import randint, uniform


my_system = ChSystem()


num_particles = 50
particle_radii = [uniform(0.05, 0.15) for _ in range(num_particles)]
particle_positions = [ChVector(uniform(-5, 5), uniform(0, 10), uniform(-5, 5)) for _ in range(num_particles)]
particle_velocities = [ChVector(uniform(-1, 1), uniform(-2, 0), uniform(-1, 1)) for _ in range(num_particles)]
particle_orientations = [ChQuaternion(uniform(0, 1), uniform(0, 1), uniform(0, 1), uniform(0, 1)) for _ in range(num_particles)]

particles = []
for i in range(num_particles):
    particle = ChBody(my_system.GetCollisionModel())
    particle.SetPos(particle_positions[i])
    particle.SetRot(particle_orientations[i])
    particle.SetMass(1.0)
    particle.SetInertiaXX(ChVector(1, 1, 1))
    particle.AddGeometry(ChSphere(particle_radii[i]))
    particle.SetBodyFixed(True)
    my_system.AddBody(particle)
    particles.append(particle)


my_application = ChirrApp(my_system)
my_application.AddTypicalSky()
my_application.AddTypicalLogo()
my_application.AddTypicalCamera(ChVector(0, 10, 0))
my_application.SetTimestep(0.01)


for i in range(num_particles):
    for j in range(i+1, num_particles):
        dist = (particles[i].GetPos() - particles[j].GetPos()).Length()
        force = (chrono.ChVector(0, -10000 * (dist**2), 0) / dist).Normalize()
        my_system.AddForce(chrono.ChForce().SetSphericalJacobian(particles[i], particles[j], force))


while my_application.GetDevice().run():
    my_system.DoStepDynamics(my_application.GetTimestep())
    my_application.DrawAll()
    my_application.DoStepRender()