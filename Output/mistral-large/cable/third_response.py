import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.n_chains = n_chains
        self.system = system
        self.mesh = mesh

        msection_cable2 = fea.ChBeamSectionCable()
        msection_cable2.SetDiameter(0.015)
        msection_cable2.SetYoungModulus(0.01e9)
        msection_cable2.SetRayleighDamping(0.0001)

        builder = fea.ChBuilderCableANCF()

        for i in range(n_chains):
            num_elements = 10 + i * 2  # Increase the number of elements with each chain

            builder.BuildBeam(
                mesh,
                msection_cable2,
                num_elements,
                chrono.ChVector3d(i * 0.2, 0, -0.1),  # Set relative starting positions
                chrono.ChVector3d((i + 1) * 0.2, 0, -0.1),  # Set relative ending positions
            )

            builder.GetLastBeamNodes().front().SetForce(chrono.ChVector3d(0, -0.7, 0))

            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)

            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(builder.GetLastBeamNodes().back(), mtruss)
            system.Add(constraint_hinge)

            # Create a box body and connect it to the beam's endpoint
            box_body = chrono.ChBodyBox(1, 1, 1, 1000)
            box_body.SetPos(chrono.ChVector3d((i + 1) * 0.2, 0, -0.1))
            system.Add(box_body)

            constraint_box = chrono.ChLinkLockLock()
            constraint_box.Initialize(box_body, builder.GetLastBeamNodes().back())
            system.Add(constraint_box)

    def PrintBodyPositions(self):
        for i in range(self.n_chains):
            box_body = self.system.Get_bodylist()[i + 1]  # Assuming the boxes are added after the mesh and truss bodies
            print(f"Chain {i + 1} box position: {box_body.GetPos()}")

# Initialize the physical system and mesh container:
sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()

# Create the model and add the mesh to the system
model = Model1(sys, mesh)
sys.Add(mesh)

# Add visualization for the FEM mesh and nodes (similar to the original code)
# ...

# Create the Irrlicht visualization for rendering
# ...

# Set solver type and settings
# ...

# Set the timestepper for the simulation
# ...

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)
    model.PrintBodyPositions()  # Print the positions of the end bodies in each chain