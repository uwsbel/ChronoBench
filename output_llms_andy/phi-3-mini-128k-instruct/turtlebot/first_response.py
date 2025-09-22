# Import necessary PyChrono and Irrlicht libraries
from pychrono.core import *
from pychrono.utils import *
from pychrono.irrlicht_utils import *

# Initialize the PyChrono environment
app = Application()
chrono_init(app)

# Create the Chrono system
chrono_sys = ChronoSystem()

# Create the ground body and set its physical properties
ground_body = chrono_sys.MakeBody()
ground_body.SetBodyFixed(True)
ground_body.SetGravity(chrono_sys.GetGravity())
ground_body.SetBodyPosition(chrono_sys.GetWorld()->GetOrigin())
ground_body.SetBodyOrientation(chrono_sys.GetWorld()->GetQuaternion())

# Add the ground body to the Chrono system
chrono_sys.AddBody(ground_body)

# Create the Turtlebot robot and set its physical properties
turtlebot = chrono_sys.MakeSphere(0.05, chrono_sys.GetWorld(), "Turtlebot")
turtlebot.SetBodyPosition(chrono_sys.GetWorld()->GetOrigin() + chrono_sys.GetWorld()->GetAxis(0, 0.5)
turtlebot.SetBodyOrientation(chrono_sys.GetWorld()->GetQuaternion())
turtlebot.SetBodyMass(1.0)
turtlebot.SetBodyInertiaXXLiquid(chrono_sys.GetWorld()->GetQuaternion())
turtlebot.SetBodyVelocity(chrono_sys.GetWorld()->GetOrigin())

# Define motor joints for left and right wheels
motor_left = chrono_sys.MakeJointRigid(chrono_sys.GetWorld(), turtlebot, chrono_sys.MakeJointSpherical(chrono_sys.GetWorld(), chrono_sys.GetWorld(), turtlebot, chrono_sys.GetWorld(), 0.0, 0.0, 0.0, 0.0)
motor_right = chrono_sys.MakeJointRigid(chrono_sys.GetWorld(), turtlebot, chrono_sys.GetWorld(), 0.0, 0.0, 0.0, 0.0)

# Set motor properties (assuming motor constants are defined)
motor_left.SetMotor(chrono_sys.GetWorld(), 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 


You are a PyChrono expert. Generate or fix a simulation script. Follow instructions carefully and return Python code inside triple backticks.


You are a PyChrono expert tasked with generating a simulation script based on the following instructions. Make sure to:
1. Initialize the PyChrono environment and core components.
2. Add the required physical systems and objects as specified.
3. Set necessary default parameters such as positions, forces, and interactions.

Instructions:
"""
Create a PyChrono simulation with a Turtlebot robot on a rigid terrain. Set up the Chrono system with gravity and a ground body, then initialize the Turtlebot with a specific position and orientation. Implement a simulation loop where the Turtlebot's motors control left and right turns at specified times, updating and rendering the robot at each timestep.
"""