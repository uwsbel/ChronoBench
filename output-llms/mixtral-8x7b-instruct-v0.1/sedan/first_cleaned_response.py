import chrono
from chrono import chrono_python as cp


my_chrono = cp.ChronoEngine()


my_chrono.Initialize(cp.ChronoSystemSolver.SIMULATION)
my_chrono.Set_G_acc(cp.Vector3d(0, -9.81, 0))


ground = my_chrono.AddBody(cp.RigidBody())
ground_shape = cp.BoxShape(ground, 10, 0.1, 10)
ground.AddAsset(ground_shape)
ground.SetPos(cp.ChVector(0, -0.5, 0))


box = my_chrono.AddBody(cp.RigidBody())
box_shape = cp.BoxShape(box, 1, 1, 1)
box.AddAsset(box_shape)
box.SetPos(cp.ChVector(0, 0, 0.5))
box.SetCollide(ground, True)


sphere = my_chrono.AddBody(cp.RigidBody())
sphere_shape = cp.SphereShape(sphere, 0.5)
sphere.AddAsset(sphere_shape)
sphere.SetPos(cp.ChVector(0, 1, 0.5))
sphere.SetCollide(box, True)
sphere.SetCollide(ground, True)


box.SetMass(10)
sphere.SetMass(1)
sphere.SetForce(cp.ChVector(0, -10, 0))


my_chrono.SetTimeStep(1e-3)
my_chrono.SetMaxNumIterations(10000)


my_chrono.RunSimulation()


my_chrono.DumpSimulationCSV("simulation_results.csv")