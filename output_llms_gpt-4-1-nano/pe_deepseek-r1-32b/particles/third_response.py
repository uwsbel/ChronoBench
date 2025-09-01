import pychrono as chrono
import pychrono.irrlicht as chronoirr
from itertools import combinations

# Create a Chrono physical system
sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create three sphere bodies with specified positions and velocities
sphere_mat = chrono.ChContactMaterialNSC()
sphere_mat.SetFriction(0.2)

# Sphere 1
sphere1 = chrono.ChBodyEasySphere(2.1, 1800, True, True, sphere_mat)
sphere1.SetPos(chrono.ChVector3d(1, 1, 0))
sphere1.SetPosDt(chrono.ChVector3d(0.5, 0, 0.1))  # Velocity (0.5, 0, 0.1)
sys.Add(sphere1)

# Sphere 2
sphere2 = chrono.ChBodyEasySphere(2.1, 1800, True, True, sphere_mat)
sphere2.SetPos(chrono.ChVector3d(-10, -10, 0))
sphere2.SetPosDt(chrono.ChVector3d(-0.5, 0, -0.1))  # Velocity (-0.5, 0, -0.1)
sys.Add(sphere2)

# Sphere 3
sphere3 = chrono.ChBodyEasySphere(2.1, 1800, True, True, sphere_mat)
sphere3.SetPos(chrono.ChVector3d(0, 20, 0))
sphere3.SetPosDt(chrono.ChVector3d(0, -0.5, 0.2))  # Velocity (0, -0.5, 0.2)
sys.Add(sphere3)

# Create an Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Three-Body Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 14, -20))
vis.AddTypicalLights()

sys.SetSolverType(chrono.ChSolver.Type_PSOR)
sys.GetSolver().AsIterative().SetMaxIterations(40)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

# Simulation loop
stepsize = 1e-2

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    for body in sys.GetBodies():
        body.EmptyAccumulators()

    G_constant = 6.674e-3  # Modified gravitational constant

    # Calculate the total kinetic energy of the system
    kinetic_energy = 0
    for body in sys.GetBodies():
        mass = body.GetMass()
        velocity = body.GetPosDt()
        kinetic_energy += 0.5 * mass * velocity.Length2()

    # Calculate the total potential energy of the system (using gravitational potential)
    potential_energy = 0
    for abodyA, abodyB in combinations(sys.GetBodies(), 2):
        D_attract = abodyB.GetPos() - abodyA.GetPos()
        r_attract = D_attract.Length()
        potential_energy += -G_constant * (abodyA.GetMass() * abodyB.GetMass()) / r_attract

    # Total energy of the system
    total_energy = kinetic_energy + potential_energy

    # Apply gravitational forces between all pairs of bodies
    mlist = list(combinations(sys.GetBodies(), 2))
    for abodyA, abodyB in mlist:
        D_attract = abodyB.GetPos() - abodyA.GetPos()
        r_attract = D_attract.Length()
        if r_attract > 0:  # Avoid division by zero
            f_attract = G_constant * (abodyA.GetMass() * abodyB.GetMass()) / (r_attract ** 2)
            F_attract = (D_attract / r_attract) * f_attract
            abodyA.AccumulateForce(F_attract, abodyA.GetPos(), False)
            abodyB.AccumulateForce(-F_attract, abodyB.GetPos(), False)

    # Print the energy values
    print(
        f"Kinetic Energy: {kinetic_energy:.6f}, Potential Energy: {potential_energy:.6f}, Total Energy: {total_energy:.6f}")

    sys.DoStepDynamics(stepsize)