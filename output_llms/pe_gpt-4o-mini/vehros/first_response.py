import os
import math
import numpy as np
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import rospy  # Import ROS library for Python
from std_msgs.msg import Float64  # Import necessary ROS message types

# Initialize ROS node
rospy.init_node('hmmwv_simulation', anonymous=True)

# Set up publishers for vehicle state
pub_velocity = rospy.Publisher('/vehicle/velocity', Float64, queue_size=10)
pub_position = rospy.Publisher('/vehicle/position', Float64, queue_size=10)

# Initialize the Chrono system
sys = chrono.ChSystemSMC()
chrono.SetChronoDataPath('/path/to/chrono/data')  # Set to your Chrono data path

# Create the HMMWV vehicle
hmmwv = veh.HMMWV()
hmmwv.SetContactMethod(chrono.ChContactMethod.SMART)
hmmwv.SetChassisCollisionType(veh.ChassisCollisionType.NONE)  # No collision for the chassis
hmmwv.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
hmmwv.Initialize()

# Set up terrain with defined friction and restitution
terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChMaterialSurfaceSMC()
patch_mat.SetFriction(0.7)  # Define friction
patch_mat.SetRestitution(0.1)  # Define restitution
terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 100, 100)  # Size of the terrain patch
terrain.Initialize()

# Initialize driver system for vehicle control
driver = veh.HMMWV_Driver(hmmwv)
driver.Initialize()

# Set up visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('HMMWV Simulation')
vis.Initialize()
vis.AddTypicalLights()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 6))
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))

# Simulation loop
time_step = 0.01  # Time step for simulation
sim_time = 0.0

while vis.Run():
    # Synchronize and advance the vehicle, terrain, and driver systems
    sys.DoStepDynamics(time_step)

    # Publish vehicle state to ROS
    velocity = hmmwv.GetVehicleSpeed()  # Get the current velocity of the vehicle
    position = hmmwv.GetVehiclePos()  # Get the current position of the vehicle

    # Publish velocity and position
    pub_velocity.publish(Float64(velocity))
    pub_position.publish(Float64(position.x))  # Publishing only x position for simplicity

    # Render the visualization
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Update simulation time
    sim_time += time_step