import pychrono as chrono
import pychrono.postprocess as postprocess
import pychrono.irrlicht as chronoirr
try:
    import pychrono.ros as ros
except:
    print("Could not import pychrono.ros.  Make sure you have it installed.")
    exit(1)

# ---------------------------------------------------------------------
#
#  Create the simulation system and add items
#

mysystem      = chrono.ChSystemNSC()
mysystem.Set_G_acc(chrono.ChVectorD(0,-9.81,0))

# Set some contact parameters
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.001)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.001)

# Create a contact material (shared among all objects)
material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.6)
material.SetDampingF(0.2)
material.SetCompliance(0.00001)
material.SetComplianceT(0.00001)

# Create the floor: a simple fixed rigid body with a collision shape
mfloor = chrono.ChBodyEasyBox(20, 1, 20, 1000, True, True, material)
mfloor.SetBodyFixed(True)
mysystem.Add(mfloor)

# Create the falling box: a rigid body with a collision shape
mbox = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 1000, True, True, material)
mbox.SetPos(chrono.ChVectorD(0, 2, 0))
mysystem.Add(mbox)

# ---------------------------------------------------------------------
#
#  Create the ROS node and handlers
#

# Create the ROS node
node = ros.ChNodeROS("chrono_sim")
node.Init()

# Custom ROS handler to publish integer messages
class MyCustomHandler(ros.ChRosPublisher):
    def __init__(self, topic_name):
        super().__init__(topic_name, "std_msgs/Int32")
        self.counter = 0

    def publish(self):
        msg = self.getMessage()
        msg.data = self.counter
        self.counter += 1
        super().publish(msg)

# Create the ROS manager
manager = ros.ChRosManager()
manager.SetNode(node)

# Clock handler to publish simulation time
clock_handler = ros.ChRosClockHandler()
manager.AddHandler(clock_handler)

# Body handler to publish body poses
body_handler = ros.ChRosBodyHandler()
body_handler.AddBody(mbox)
manager.AddHandler(body_handler)

# Transform handler to publish transforms
transform_handler = ros.ChRosTransformHandler()
transform_handler.AddTransform(mbox, "box_frame")
manager.AddHandler(transform_handler)

# Register the custom handler
custom_handler = MyCustomHandler("custom_topic")
manager.AddHandler(custom_handler)

# ---------------------------------------------------------------------
#
#  Run the simulation loop
#

# Simulation loop
application = chronoirr.ChIrrApp(mysystem, 'PyChrono example', chronoirr.dimension2du(1024,768))
application.AddTypicalSky()
application.AddTypicalLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
application.AddTypicalCamera(chronoirr.vector3df(2,2,2))
application.AddTypicalLights()

application.AssetBindAll();
application.AssetUpdateAll();

# Change the solver form the default SOR to a more precise solver
mysystem.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
mysystem.SetSolverMaxIterations(100)

application.SetTimestep(0.01)
application.SetTryRealtime(True)

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()

    # Update ROS communication
    manager.Update()