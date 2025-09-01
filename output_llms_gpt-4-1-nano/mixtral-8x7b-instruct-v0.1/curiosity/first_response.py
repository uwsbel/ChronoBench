"""
PyChrono simulation script for a simple physical system

This script creates a physical system with a ground plane, a box, and a sphere.
The box is placed on the ground plane, and the sphere is placed above the box.
A constant downward force is applied to the sphere.

"""

# Import necessary modules
import chrono
from chrono import chrono_python as cp

# Initialize the PyChrono environment
my_chrono = cp.ChronoEngine()

# Add the required physical systems
my_chrono.Initialize(cp.ChronoSystemSolver.SIMULATION)
my_chrono.Set_G_acc(cp.Vector3d(0, -9.81, 0))

# Create ground plane
ground = my_chrono.AddBody(cp.RigidBody())
ground_shape = cp.BoxShape(ground, 10, 0.1, 10)
ground.AddAsset(ground_shape)

# Create box
box = my_chrono.AddBody(cp.RigidBody())
box_shape = cp.BoxShape(box, 1, 1, 1)
box.AddAsset(box_shape)
box.SetPos(cp.ChVectorD(0, 0.5, 0))
box.SetRot(cp.ChQuaternionD(1, 0, 0, 0))

# Create sphere
sphere = my_chrono.AddBody(cp.RigidBody())
sphere_shape = cp.SphereShape(sphere, 0.5)
sphere.AddAsset(sphere_shape)
sphere.SetPos(cp.ChVectorD(0, 1.5, 0))
sphere.SetRot(cp.ChQuaternionD(1, 0, 0, 0))

# Add constant downward force to the sphere
force_maker = cp.UniformForce(my_chrono.GetSystem())
force_maker.Set_force(cp.ChVectorD(0, -10, 0))
sphere.AddAsset(force_maker)
sphere.AddForce(force_maker)

# Set necessary default parameters
my_chrono.Set_timestep(1e-4)
my_chrono.Set_playercollision(True)

# Run the simulation
my_chrono.DoSimulation(10, 1e-4)

# Print the final positions of the box and sphere
print("Final position of the box: ", box.GetPos())
print("Final position of the sphere: ", sphere.GetPos())