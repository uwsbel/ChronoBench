import pychrono as chrono
import pychrono.irrlicht as chronoirr

# Create a Chrono physical system
sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
coll = sys.GetCollisionSystem()

# Create contact material
sphere_mat = chrono.ChContactMaterialNSC()
sphere_mat.SetFriction(0.2)

# Create three sphere bodies
msphereBody1 = chrono.ChBodyEasySphere(2.1, 1800, True, True, sphere_mat)
msphereBody1.SetPos(chrono.ChVector3d(1, 1, 0))
msphereBody1.SetPosDt(chrono.ChVector3d(0.5, 0, 0.1))  # Set initial velocity
msphereBody1.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(msphereBody1)

msphereBody2 = chrono.ChBodyEasySphere(2.1, 1800, True, True, sphere_mat)
msphereBody2.SetPos(chrono.ChVector3d(-10, -10, 0))
msphereBody2.SetPosDt(chrono.ChVector3d(-0.5, 0, -0.1))  # Set initial velocity
msphereBody2.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(msphereBody2)

msphereBody3 = chrono.ChBodyEasySphere(2.1, 1800, True, True, sphere_mat)
msphereBody3.SetPos(chrono.ChVector3d(0, 20, 0))
msphereBody3.SetPosDt(chrono.ChVector3d(0, -0.5, 0.2))  # Set initial velocity
msphereBody3.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(msphereBody3)

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Three-body simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 14, -20))
vis.AddTypicalLights()

sys.SetSolverType(chrono.ChSolver.Type_PSOR)
sys.GetSolver().AsIterative().SetMaxIterations(40)

# Simulation loop
stepsize = 1e-2
G_constant = 6.674e-3  # Gravitational constant

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Calculate the total kinetic energy of the system
    kinetic_energy = 0
    for body in sys.GetBodies():
        mass = body.GetMass()
        velocity = body.GetPosDt()
        kinetic_energy += 0.5 * mass * velocity.Length2()

    # Calculate the total potential energy of the system (using gravitational potential)
    potential_energy = 0
    for abodyA, abodyB in zip(list(sys.GetBodies())[:-1], list(sys.GetBodies())[1:]):
        D_attract = abodyB.GetPos() - abodyA.GetPos()
        r_attract = D_attract.Length()
        potential_energy += -G_constant * (abodyA.GetMass() * abodyB.GetMass()) / r_attract

    # Total energy of the system
    total_energy = kinetic_energy + potential_energy

    bodies = list(sys.GetBodies())
    for i, abodyA in enumerate(bodies):
        for abodyB in bodies[i+1:]:
            D_attract = abodyB.GetPos() - abodyA.GetPos()
            r_attract = D_attract.Length()
            f_attract = G_constant * (abodyA.GetMass() * abodyB.GetMass()) / (r_attract ** 2)
            F_attract = (D_attract / r_attract) * f_attract
            abodyA.AccumulateForce(F_attract, abodyA.GetPos(), False)
            abodyB.AccumulateForce(-F_attract, abodyB.GetPos(), False)

    # Print the energy values
    print(f"Kinetic Energy: {kinetic_energy:.6f}, Potential Energy: {potential_energy:.6f}, Total Energy: {total_energy:.6f}")

    sys.DoStepDynamics(stepsize)
    for body in sys.GetBodies():
        body.EmptyAccumulators()

# No code needs to be returned outside the code block.