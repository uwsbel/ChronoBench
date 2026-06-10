import pychrono as chrono
import pychrono.irrlicht as chronoirr
from itertools import combinations


# ---------------------------------------------------------------------
# Create a Chrono physical system
# ---------------------------------------------------------------------
sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

# Solver settings
sys.SetSolverType(chrono.ChSolver.Type_PSOR)
sys.GetSolver().AsIterative().SetMaxIterations(40)


# ---------------------------------------------------------------------
# Contact material
# ---------------------------------------------------------------------
sphere_mat = chrono.ChContactMaterialNSC()
sphere_mat.SetFriction(0.2)


# ---------------------------------------------------------------------
# Create three spheres for the three-body simulation
# ---------------------------------------------------------------------
sphere_radius = 2.1
sphere_density = 1800

sphere1 = chrono.ChBodyEasySphere(
    sphere_radius,
    sphere_density,
    True,   # visualization
    True,   # collision
    sphere_mat
)
sphere1.SetPos(chrono.ChVector3d(1, 1, 0))
sphere1.SetPosDt(chrono.ChVector3d(0.5, 0, 0.1))
sphere1.GetVisualShape(0).SetTexture(
    chrono.GetChronoDataFile("textures/concrete.jpg")
)
sphere1.SetUseGyroTorque(False)
sys.Add(sphere1)

sphere2 = chrono.ChBodyEasySphere(
    sphere_radius,
    sphere_density,
    True,
    True,
    sphere_mat
)
sphere2.SetPos(chrono.ChVector3d(-10, -10, 0))
sphere2.SetPosDt(chrono.ChVector3d(-0.5, 0, -0.1))
sphere2.GetVisualShape(0).SetTexture(
    chrono.GetChronoDataFile("textures/bluewhite.png")
)
sphere2.SetUseGyroTorque(False)
sys.Add(sphere2)

sphere3 = chrono.ChBodyEasySphere(
    sphere_radius,
    sphere_density,
    True,
    True,
    sphere_mat
)
sphere3.SetPos(chrono.ChVector3d(0, 20, 0))
sphere3.SetPosDt(chrono.ChVector3d(0, -0.5, 0.2))
sphere3.GetVisualShape(0).SetTexture(
    chrono.GetChronoDataFile("textures/bluewhite.png")
)
sphere3.SetUseGyroTorque(False)
sys.Add(sphere3)

bodies = [sphere1, sphere2, sphere3]


# ---------------------------------------------------------------------
# Irrlicht visualization
# ---------------------------------------------------------------------
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Three-body PyChrono simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 25, -45), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()


# ---------------------------------------------------------------------
# Simulation parameters
# ---------------------------------------------------------------------
stepsize = 1e-2
G_constant = 6.674e-3
min_distance = 1e-8


# ---------------------------------------------------------------------
# Simulation loop
# ---------------------------------------------------------------------
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Clear previously accumulated forces
    for body in bodies:
        body.EmptyAccumulators()

    # Compute kinetic energy
    kinetic_energy = 0.0
    for body in bodies:
        mass = body.GetMass()
        velocity = body.GetPosDt()
        kinetic_energy += 0.5 * mass * velocity.Length2()

    # Compute gravitational potential energy
    potential_energy = 0.0
    for body_a, body_b in combinations(bodies, 2):
        d_vec = body_b.GetPos() - body_a.GetPos()
        distance = d_vec.Length()

        if distance > min_distance:
            potential_energy += (
                -G_constant * body_a.GetMass() * body_b.GetMass() / distance
            )

    # Apply pairwise gravitational forces
    for body_a, body_b in combinations(bodies, 2):
        d_vec = body_b.GetPos() - body_a.GetPos()
        distance = d_vec.Length()

        if distance > min_distance:
            force_magnitude = (
                G_constant * body_a.GetMass() * body_b.GetMass()
                / (distance * distance)
            )

            force_direction = d_vec / distance
            force = force_direction * force_magnitude

            body_a.AccumulateForce(force, body_a.GetPos(), False)
            body_b.AccumulateForce(force * -1, body_b.GetPos(), False)

    total_energy = kinetic_energy + potential_energy

    print(
        f"Kinetic Energy: {kinetic_energy:.6f}, "
        f"Potential Energy: {potential_energy:.6f}, "
        f"Total Energy: {total_energy:.6f}"
    )

    sys.DoStepDynamics(stepsize)