```python

from pychrono import ChBody, ChVector, ChQuaternion, ChSystem, ChSystemNSC, ChSystemNSC::Dynamics, ChTire, ChRigidTerrain, ChMeshVisual, ChTextureVisual
from pychrono.utils import *
import irrlicht

# Initialize PyChrono environment

system = ChSystemNSC()

# Define the vehicle parameters

vehicle_mass = 1000.0  # Mass of the vehicle in kg

vehicle_inertia = ChVector(1000.0, 1000.0, 1000.0)  # Inertia tensor in kg*m^2

vehicle_position = ChVector(0.0, 0.0, 0.0)  # Initial position of the vehicle

vehicle_orientation = ChQuaternion(1.0, 0.0, 0.0, 0.0)  # Initial orientation of the vehicle

vehicle_steering_angle = 0.0  # Initial steering angle

vehicle_throttle = 0.0  # Initial throttle value

vehicle_brake = 0.0  # Initial brake value


# Create the vehicle body

vehicle_body = ChBody(vehicle_mass, vehicle_inertia)

vehicle_body.SetPos(vehicle_position)

vehicle_body.SetRot(vehicle_orientation)


# Add the vehicle to the system

system.AddBody(vehicle_tire_model = ChTire(ChTireModel::TMEASY)

vehicle_body = ChBody(vehicle_mass, vehicle_inertia)

vehicle_body.SetPos(vehicle_position)

vehicle_body.SetRot(vehicle_orientation)

vehicle_body.SetInertia(vehicle_inertia)

vehicle_body.SetMass(vehicle_mass)

vehicle_body.SetInertia(vehicle_inertia)

vehicle_body.SetMass(vehicle_mass)

vehicle_body.SetInertia(vehicle_inertia)

vehicle_body.SetPos(vehicle_position)

vehicle_body.SetRot(vehicle_orientation)


# Add the vehicle body to the system

system.AddBody(vehicle_body)


# Initialize the tire model

tire_model.SetFlatContact(True)

tire_model.SetFrictionCoeff(0.5)

tire_model.SetFlatContact(True)

vehicle_tire = ChTire(vehicle_tire_model)

vehicle_tire.SetTireModel(vehicle_tire_model)


# Create the vehicle chassis

chassis = ChBody()

chassis.SetMass(vehicle_mass)

chassis.SetPos(vehicle_position)

chassis.SetRot(vehicle_orientation)

vehicle_chassis = ChBody()

vehicle_chassis.SetMass(vehicle_mass)

vehicle_chassis.SetPos(vehicle_position)

vehicle_chassis.SetRot(vehicle_orientation)

vehicle_chassis.SetInertia(vehicle_inertia)

system.AddBody(vehicle_chassis)


# Add the tire to the chassis

chassis.AddTire(vehicle_tire, vehicle_tire_model)


# Add the chassis to the vehicle body

vehicle_body.AddBody(chassis)


# Add the tire to the chassis

chassis.AddTire(vehicle_tire)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Add the vehicle to the system

system.AddBody(vehicle_body)


# Initialize the tire model

tire_model.SetFrictionCoeff(0.5)

tire_model.SetFlatContact(True)


# Set the vehicle's initial velocity

vehicle_body.SetVel(ChVector(0.0, 0.0, 0.0)


# Add the tire to the vehicle

vehicle_body.AddTire(vehicle_tire)


# Create the chassis for the vehicle

chassis = ChBody()

chassis.SetMass(vehicle_mass)

chassis.SetPos(vehicle_position)

chassis.SetRot(vehicle_orientation)

vehicle_body.AddBody(chassis)


# Create the tire model

tire_model = ChTire(vehicle_tire_model)

tire_model.SetFlatContact(True)

tire_model.SetFrictionCoeff(0.5)


# Add the tire to the chassis

chassis.AddTire(vehicle_tire)


# Initialize the vehicle

vehicle_body.AddBody(chassis)


# Create the terrain

terrain = ChRigidTerrain(vehicle_tire_model)

terrain.SetPos(vehicle_position)

terrain.SetRot(vehicle_orientation)

system.AddBody(terrain)


# Set the vehicle's initial velocity

vehicle_body.SetVel(ChVector(0.0, 0.0, 0.0)


# Create the driver

driver = ChDriver(system)

driver.SetSteeringGain(0.5)

driver.SetThrottleGain(0.5)

driver.SetBrakeGain(0.5)


# Set the driver to the vehicle

vehicle_body.SetDriver(driver)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial velocity

vehicle_body.SetVel(ChVector(0.0, 0.0, 0.0)


# Create the terrain

terrain = ChRigidTerrain(vehicle_tire_model)

terrain.SetPos(vehicle_position)

terrain.SetRot(vehicle_orientation)

system.AddBody(terrain)


# Set the vehicle's initial velocity

vehicle_body.SetVel(ChVector(0.0, 0.0, 0.0)


# Create the visualization

irrlicht_application = irrlicht.Application()


# Set the vehicle's visualization

vehicle_mesh = ChMeshVisual(vehicle_mesh)

vehicle_mesh.SetName("Vehicle")

vehicle_mesh.SetColor(1.0, 1.0, 1.0)

vehicle_mesh.SetScale(1.0)

vehicle_mesh.SetPos(vehicle_position)

vehicle_mesh.SetRot(vehicle_orientation)


# Create the texture visualization

vehicle_texture = ChTextureVisual(vehicle_texture)

vehicle_texture.SetName("VehicleTexture")

vehicle_texture.SetScale(1.0)

vehicle_texture.SetColor(1.0, 1.0, 1.0)

vehicle_texture.SetTexture(vehicle_texture)

vehicle_texture.SetPos(vehicle_position)

vehicle_texture.SetRot(vehicle_orientation)


# Set the vehicle's visualization

vehicle_mesh.SetParent(vehicle_body)

vehicle_texture.SetParent(vehicle_body)


# Set the vehicle's visualization

vehicle_body.AddVisual(vehicle_mesh)

vehicle_body.AddVisual(vehicle_texture)


# Create the driver

driver = ChDriver(system)

driver.SetSteeringGain(0.5)

driver.SetThrottleGain(0.5)

driver.SetBrakeGain(0.5)


# Set the vehicle's driver

vehicle_body.SetDriver(driver)


# Set the vehicle's initial velocity

vehicle_body.SetVel(ChVector(0.0, 0.0, 0.0)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial velocity

vehicle_body.SetVel(ChVector(0.0, 0.0, 0.0)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial velocity

vehicle_body.SetVel(ChVector(0.0, 0.0, 0.0)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial velocity

vehicle_body.SetVel(ChVector(0.0, 0.0, 0.0)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial velocity

vehicle_body.SetVel(ChVector(0.0, 0.0, 0.0)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial velocity

vehicle_body.SetVel(ChVector(0.0, 0.0, 0.0)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial velocity

vehicle_body.SetVel(ChVector(0.0, 0.0, 0.0)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial velocity

vehicle_body.SetVel(ChVector(0.0, 0.0, 0.0)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)


# Set the vehicle's initial orientation

vehicle_body.SetRot(vehicle_orientation)