import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# ----------------------------------------------------------------------------

class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.n_chains = n_chains
        self.mesh = mesh
        self.system = system

        # Create a section, i.e. define thickness and material properties for the cable beam
        msection_cable2 = fea.ChBeamSectionCable()
        msection_cable2.SetDiameter(0.015)  # Set the diameter of the cable section to 15 mm
        msection_cable2.SetYoungModulus(0.01e9)  # Set the Young's modulus of the cable section (0.01 GPa)
        msection_cable2.SetRayleighDamping(0.0001)  # Set Rayleigh damping to zero for this section

        # Create a ChBuilderCableANCF helper object to facilitate the creation of ANCF beams
        builder = fea.ChBuilderCableANCF()
        # Use BuildBeam to create a beam structure consisting of ANCF elements:
        for i in range(self.n_chains):
            offset = i * 0.2  # offset for the start point of each chain
            builder.BuildBeam(
                self.mesh,  # The mesh to which the created nodes and elements will be added
                msection_cable2,  # The beam section properties to use
                15,  # Number of ANCF elements to create along the beam
                chrono.ChVector3d(-0.1 + offset, 0.5, -0.1),  # Starting point ('A' point) of the beam
                chrono.ChVector3d(-0.1 + offset, 0.5, 0.5)  # Ending point ('B' point) of the beam
            )

            # Apply boundary conditions and loads:
            # Retrieve the end nodes of the beam and apply load/constraints
            end_nodes = builder.GetLastBeamNodes()
            end_node = end_nodes.back()
            end_node.SetForce(chrono.ChVector3d(0, -0.7, 0))  # Apply forces to the front node

            # Create a truss body (a fixed reference frame in the simulation)
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)  # Fix the truss body

            # Create and initialize a hinge constraint to fix beam's end point to the truss
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(end_node, mtruss)
            self.system.Add(constraint_hinge)  # Add the constraint to the system

            # Create a box body
            box = chrono.ChBody()
            box.SetMass(0)
            box.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
            box.SetPos(end_node.GetPos())
            box.SetFixed(False)
            box.SetName("box")
            self.system.Add(box)

            # Create a revolute constraint between the box and the end node
            rev = fea.ChLinkLockRevolute()
            rev.Initialize(box, end_node)
            self.system.Add(rev)

        # Create a truss body (a fixed reference frame in the simulation)
        mtruss = chrono.ChBody()
        mtruss.SetFixed(True)  # Fix the truss body

        # Create and initialize a hinge constraint to fix beam's end point to the truss
        constraint_hinge = fea.ChLinkNodeFrame()
        constraint_hinge.Initialize(builder.GetFirstBeamNode(), mtruss)
        self.system.Add(constraint_hinge)  # Add the constraint to the system

    def PrintBodyPositions(self):
        end_bodies = []
        for i in range(self.n_chains):
            offset = i * 0.2
            end_body = chrono.CastContactBodyToChBody(self.mesh.GetNthBody(2 + i * 15 + 1))
            end_bodies.append(end_body)
        print("Positions of end bodies:")
        for body in end_bodies:
            print(body.GetPos())

# ----------------------------------------------------------------------------

def main():
    # Create the physical system
    sys = chrono.ChSystemSMC()
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

    # Create the mesh container
    mesh = fea.ChMesh()

    # Create the FEA model
    model = Model1(sys, mesh)
    sys.GetSolver().AsIterative().SetMaxIterations(100)

    # Create the visualization
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024,768)
    vis.SetWindowTitle('FEA cables')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddTypicalLights()
    vis.AddCamera(chrono.ChVector3d(0,0.6,-1))
    vis.AddTypicalOrbitCamera(trackPoint=chrono.ChVector3d(0,0.5,0.5))

    # Set the time response for rotating camera tracks to changes in the viewed scene.
    camTrackRotateTime = 2.0  # time to go from 0 to +1 (or from 0 to -1)
    camTrackZoomTime = 1.0   # time to go from 0 to +1
    vis.SetCameraTrackSpeeds(camTrackRotateTime, camTrackZoomTime)

    # Simulation loop
    time = 0
    time_step = 1e-3
    time_end = 10

    # Initialize simulation frame rate
    frame_rate = 0

    # Set data path to default location for this demo
    # (make sure demo is run from same directory as demoData directory)
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())

    # Initialize simulation frame counter
    render_frames = 0

    while vis.Run() :
        time = sys.GetChTime()

        # Render scene and output POV-Ray data
        if (render_frames % 10 == 0) :
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
        render_frames += 1

        # Get the current frame number
        frame_number = int(time / time_step)

        # Break if going backwards in time
        if (frame_number <= 0) :
            break

        # Render scene and output POV-Ray data
        if (frame_number % 20 == 0) :
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        # Advance simulation for one timestep for all bodies in the system
        sys.DoStepDynamics(time_step)

        # Advance state of the visual system (process events from the main system)
        vis.Advance(time_step)

        # Increment frame number
        frame_number += 1

    return 0


# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')
# In this demo, we use the Irrlicht visualization system.
# The FEA model is created using the ChBuilderCableANCF class, which is a utility class that helps
# create the mesh and the ChLinkNodeFrame constraints for the ANCF beam model.
# The demo shows how to create multiple chains of beams and how to visualize the FEA mesh with
# various visualization types (e.g. wireframe, mesh, glyphs for nodes, etc.).
# The demo also shows how to access and print the positions of the end bodies of the beams at each
# step of the simulation.
main()