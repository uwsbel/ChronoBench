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
ground.SetPos(cp.ChVector(0, -0.5, 0))

# Create box
box = my_chrono.AddBody(cp.RigidBody())
box_shape = cp.BoxShape(box, 1, 1, 1)
box.AddAsset(box_shape)
box.SetPos(cp.ChVector(0, 0, 0.5))
box.SetCollide(ground, True)

# Create sphere
sphere = my_chrono.AddBody(cp.RigidBody())
sphere_shape = cp.SphereShape(sphere, 0.5)
sphere.AddAsset(sphere_shape)
sphere.SetPos(cp.ChVector(0, 1, 0.5))
sphere.SetCollide(box, True)
sphere.SetCollide(ground, True)

# Set default parameters
box.SetMass(10)
sphere.SetMass(1)
sphere.SetForce(cp.ChVector(0, -10, 0))

# Set up the simulation
my_chrono.SetTimeStep(1e-3)
my_chrono.SetMaxNumIterations(10000)

# Run the simulation
my_chrono.RunSimulation()

# Output the simulation results
my_chrono.DumpSimulationCSV("simulation_results.csv")