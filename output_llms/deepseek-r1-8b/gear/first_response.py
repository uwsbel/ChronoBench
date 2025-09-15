import pychrono as pc
import pychrono.guitools as gui
from pychrono.objects import *
from pychrono.mesh import *
from pychrono.dynamics import *
from pychronovisualization import *
from pychronoinput import *
from pychronomaterials import *

# Initialize PyChrono environment
pc.setGlobalCoordinateSystem(pc.COORDINATE_SYSTEM 世界)

# Create a scene
scene = pc.Scene()
scene.setAmbientLightColor(1, 1, 1)  # Set ambient light for visualization

# Create a ground plane
ground = pc.RigidBody()
ground.setMass(0)
ground.setShape(pc.Sphere(0, 0, 0, 1, 1, 1))  # Create a ground plane using a sphere
ground.setPosition(pc.Vector(0, 0, -1))  # Position the ground plane at z = -1

# Create a camera
camera = pc.Camera()
camera.setPos(pc.Vector(0, 0, 5))  # Position the camera at (0, 0, 5)

# Create a fixed truss
truss = pc.RigidBody()
truss.setMass(0)
truss.setShape(pc.Box(0.5, 0.5, 0.5))  # Create a box for the truss
truss.setPosition(pc.Vector(0, 0, 0))  # Position the truss at the origin

# Create a rotating bar
rotating_bar = pc.RigidBody()
rotating_bar.setMass(0)
rotating_bar.setShape(pc.Sphere(0.5, 0.5, 0.5))  # Create a sphere for the rotating bar
rotating_bar.setPosition(pc.Vector(1, 0, 0))  # Position the rotating bar at (1, 0, 0)

# Create a gear motor
gear_motor = pc.Motor()
gear_motor.setGearRatio(10)  # Set a high gear ratio to maintain constant rotation speed

# Create two gears
gear1 = pc.Gear()
gear1.setNumber Teeth(20)
gear1.setRadius(0.5)
gear1.setPosition(pc.Vector(0, 0, 0))

gear2 = pc.Gear()
gear2.setNumber Teeth(20)
gear2.setRadius(0.6)
gear2.setPosition(pc.Vector(0, 0, 0))

# Create a joint for the rotating bar
joint_rotating_bar = pc.FixedJoint()
joint_rotating_bar.setAttachPoint1(rotating_bar, pc.Vector(0, 0, 0))
joint_rotating_bar.setAttachPoint2(gear_motor, pc.Vector(0, 0, 0))
joint_rotating_bar.setAxis(pc.Vector(0, 0, 1))  # Rotate around the z-axis

# Create a joint for the first gear
joint_gear1 = pc.EpicyclicJoint()
joint_gear1.setAttachPoint1(gear1, pc.Vector(0, 0, 0))
joint_gear1.setAttachPoint2(rotating_bar, pc.Vector(0, 0, 0))
joint_gear1.setAxis(pc.Vector(0, 0, 1))  # Rotate around the z-axis
joint_gear1.setNumber Teeth(20)
joint_gear1.setRadius1(gear1.getRadius())
joint_gear1.setRadius2(gear_motor.getRadius())

# Create a joint for the second gear
joint_gear2 = pc.EpicyclicJoint()
joint_gear2.setAttachPoint1(gear2, pc.Vector(0, 0, 0))
joint_gear2.setAttachPoint2(rotating_bar, pc.Vector(0, 0, 0))
joint_gear2.setAxis(pc.Vector(0, 0, 1))  # Rotate around the z-axis
joint_gear2.setNumber Teeth(20)
joint_gear2.setRadius1(gear2.getRadius())
joint_gear2.setRadius2(gear_motor.getRadius())

# Add the rigid bodies to the scene
scene.add(gear_motor)
scene.add(rotating_bar)
scene.add(truss)
scene.add(gear1)
scene.add(gear2)
scene.add(joint_gear1)
scene.add(joint_gear2)
scene.add(joint_rotating_bar)

# Visualize the setup using Irrlicht
renderer = pc.Renderer()
renderer.setClearColor(0.5, 0.5, 0.5)  # Set a light gray background
renderer.setCamera(camera)
renderer.setLightDirection(pc.Vector(0, 0, 1))  # Add a light from above

# Add materials
truss_material = pc.Material()
truss_material.setDiffuseColor(0.5, 0.5, 0.5)  # Gray material
rotating_bar_material = pc.Material()
rotating_bar_material.setDiffuseColor(1, 0, 0)  # Red material
gear_material = pc.Material()
gear_material.setDiffuseColor(0.5, 0.5, 0.5)  # Gray material

# Assign materials to the rigid bodies
truss.setMaterial(truss_material)
rotating_bar.setMaterial(rotating_bar_material)
gear1.setMaterial(gear_material)
gear2.setMaterial(gear_material)

# Add the visualization
visualization = pc.Visualization(scene, renderer)

# Set up the simulation
simulation = pc.Simulation(scene, 1e-10)  # Set a very small time step for simulation

# Add forces
gravity = pc.Force()
gravity.setGravity(pc.Vector(0, 0, -9.81))
simulation.addForce(gear1, gravity)
simulation.addForce(gear2, gravity)
simulation.addForce(rotating_bar, gravity)
simulation.addForce(truss, gravity)

# Add the visualization callback
def draw():
    visualization.render()

# Run the simulation
simulation.run(0, 10)  # Run the simulation for 10 seconds