import pychrono as chrono
import pychrono.core as core
import pychrono.physics as physics


app = chrono.Application()


world = chrono.World()
app.add_world(world)


ground = chrono.Plane(
    chrono.Vector3(0, 0, 0),
    chrono.Vector3(10, 10, 0),
    chrono.Vector3(0, 0, 10),
    chrono.Vector3(0, 0, 0),
)
world.add_object(ground)


car = chrono.RigidBody(
    chrono.Vector3(0, 0, 0),
    chrono.Vector3(1, 0, 0),
    chrono.Vector3(0, 1, 0),
    chrono.Vector3(0, 0, 0),
)
world.add_object(car)


car.set_mass(100)
car.set_inertia(chrono.Vector3(0, 0, 0))
car.set_position(chrono.Vector3(0, 0, 0))
car.set_velocity(chrono.Vector3(0, 0, 0))


force = chrono.Force(chrono.Vector3(0, 10, 0))
car.add_force(force)


app.run()