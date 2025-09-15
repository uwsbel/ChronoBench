import pychrono as chrono
import pychrono.irrlicht as chronoirr
from itertools import combinations

class MyCreatorForAll(chrono.ChRandomShapeCreator_AddBodyCallback):
    def __init__(self, vis, coll):
        super().__init__()
        self.vis = vis
        self.coll = coll

    def OnAddBody(self, body, coords, creator):
        body.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))
        self.vis.BindItem(body)
        self.coll.BindItem(body)
        body.SetUseGyroTorque(False)

# Create a Chrono physical system
sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
coll = sys.GetCollisionSystem()

# Create three spheres with specified positions and velocities
sphere_mat = chrono.ChContactMaterialNSC()
sphere_mat.SetFriction(0.2)

# Sphere 1
msphereBody1 = chrono.ChBodyEasySphere(2.1, 1800, True, True, sphere_mat)
msphereBody1.SetPos(chrono.ChVector3d(1, 1, 0))
msphereBody1.SetPosDt(chrono.ChVector3d(0.5, 0, 0.1))  # Initial velocity
msphereBody1.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(msphereBody1)

# Sphere 2
msphereBody2 = chrono.ChBodyEasySphere(2.1, 1800, True, True, sphere_mat)
msphereBody2.SetPos(chrono.ChVector3d(-10, -10, 0))
msphereBody2.SetPosDt(chrono.ChVector3d(-0.5, 0, -0.1))  # Initial velocity
msphereBody2.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(msphereBody2)

# Sphere 3
msphereBody3 = chrono.ChBodyEasySphere(2.1, 1800, True, True, sphere_mat)
msphereBody3.SetPos(chrono.ChVector3d(0, 20, 0))
msphereBody3.SetPosDt(chrono.ChVector3d(0, -0.5, 0.2))  # Initial velocity
msphereBody3.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(msphereBody3)

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Three-Body Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 14, -20))
vis.AddTypicalLights()

# Register the callback for all bodies
mcreation_callback = MyCreatorForAll(vis, coll)

# Simulation settings
sys.SetSolverType(chrono.ChSolver.Type_PSOR)
sys.GetSolver().AsIterative().SetMaxIterations(40)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))  # No external gravity

# Reduced gravitational constant for simulation purposes
G_constant = 6.674e-3

# Simulation loop
stepsize = 1e-2

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Clear accumulators
    for body in sys.GetBodies():
        body.EmptyAccumulators()

    # Calculate kinetic and potential energy
    kinetic_energy = 0
    for body in sys.GetBodies():
        mass = body.GetMass()
        velocity = body.GetPosDt()
        kinetic_energy += 0.5 * mass * velocity.Length2()

    potential_energy = 0
    for abodyA, abodyB in combinations(sys.GetBodies(), 2):
        D_attract = abodyB.GetPos() - abodyA.GetPos()
        r_attract = D_attract.Length()
        potential_energy += -G_constant * (abodyA.GetMass() * abodyB.GetMass()) / r_attract

    # Apply gravitational forces
    for abodyA, abodyB in combinations(sys.GetBodies(), 2):
        D_attract = abodyB.GetPos() - abodyA.GetPos()
        r_attract = D_attract.Length()
        if r_attract > 0:  # Avoid division by zero
            f_attract = G_constant * (abodyA.GetMass() * abodyB.GetMass()) / (r_attract ** 2)
            F_attract = (D_attract / r_attract) * f_attract
            abodyA.AccumulateForce(F_attract, abodyA.GetPos(), False)
            abodyB.AccumulateForce(-F_attract, abodyB.GetPos(), False)

    # Print energy values
    total_energy = kinetic_energy + potential_energy
    print(
        f"Kinetic Energy: {kinetic_energy:.6f}, Potential Energy: {potential_energy:.6f}, Total Energy: {total_energy:.6f}")

    sys.DoStepDynamics(stepsize)