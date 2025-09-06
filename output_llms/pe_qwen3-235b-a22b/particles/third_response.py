import pychrono as chrono
import pychrono.irrlicht as chronoirr
from itertools import combinations

# Create a Chrono physical system
sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Configure gravitational material
sphere_mat = chrono.ChContactMaterialNSC()
sphere_mat.SetFriction(0.2)

# Create Sphere 1 (original)
sphere1 = chrono.ChBodyEasySphere(2.1, 1800, True, True, sphere_mat)
sphere1.SetPos(chrono.ChVector3d(1, 1, 0))
sphere1.SetPos_dt(chrono.ChVector3d(0.5, 0, 0.1))  # Initial velocity
sphere1.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(sphere1)

# Create Sphere 2
sphere2 = chrono.ChBodyEasySphere(2.1, 1800, True, True, sphere_mat)
sphere2.SetPos(chrono.ChVector3d(-10, -10, 0))
sphere2.SetPos_dt(chrono.ChVector3d(-0.5, 0, -0.1))  # Initial velocity
sphere2.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(sphere2)

# Create Sphere 3
sphere3 = chrono.ChBodyEasySphere(2.1, 1800, True, True, sphere_mat)
sphere3.SetPos(chrono.ChVector3d(0, 20, 0))
sphere3.SetPos_dt(chrono.ChVector3d(0, -0.5, 0.2))  # Initial velocity
sphere3.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(sphere3)

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Three-Body Gravitational Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 14, -20))
vis.AddTypicalLights()

# Configure solver
sys.SetSolverType(chrono.ChSolver.Type_PSOR)
sys.GetSolver().AsIterative().SetMaxIterations(40)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

# Simulation loop
stepsize = 1e-2

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Reset forces
    for body in sys.GetBodies():
        body.EmptyAccumulators()

    # Gravitational constant (scaled for simulation)
    G_constant = 6.674e-3

    # Calculate energies
    kinetic_energy = 0
    potential_energy = 0
    
    # Get all body pairs
    body_pairs = list(combinations(sys.GetBodies(), 2))
    
    # Calculate potential energy
    for bodyA, bodyB in body_pairs:
        distance = bodyB.GetPos() - bodyA.GetPos()
        r = distance.Length()
        potential_energy += -G_constant * (bodyA.GetMass() * bodyB.GetMass()) / r

    # Apply gravitational forces
    for bodyA, bodyB in body_pairs:
        distance = bodyB.GetPos() - bodyA.GetPos()
        r = distance.Length()
        
        if r > 1e-6:  # Avoid division by zero
            force_magnitude = G_constant * (bodyA.GetMass() * bodyB.GetMass()) / (r ** 2)
            force_vector = (distance / r) * force_magnitude
            
            # Apply Newton's third law
            bodyA.AccumulateForce(force_vector, bodyA.GetPos(), False)
            bodyB.AccumulateForce(-force_vector, bodyB.GetPos(), False)

    # Calculate kinetic energy
    for body in sys.GetBodies():
        velocity = body.GetPosDt()
        kinetic_energy += 0.5 * body.GetMass() * velocity.Length2()

    # Print energy values
    print(f"Kinetic Energy: {kinetic_energy:.6f}, Potential Energy: {potential_energy:.6f}, Total Energy: {kinetic_energy + potential_energy:.6f}")

    # Advance simulation
    sys.DoStepDynamics(stepsize)