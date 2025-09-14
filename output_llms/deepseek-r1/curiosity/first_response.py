import os
import math
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
from pychrono.robot import curiosity

# Initialize Chrono system
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create rigid terrain
terrain = chrono.ChBody()
terrain.SetBodyFixed(True)
terrain.SetPos(chrono.ChVectorD(0, -0.2, 0))

# Collision and visualization for terrain
terrain_shape = chrono.ChBoxShape()
terrain_shape.GetBoxGeometry().Size = chrono.ChVectorD(10, 0.2, 10)
terrain.AddVisualShape(terrain_shape)

terrain_collision = chrono.ChCollisionModelBullet()
terrain_collision.AddBox(0.5, 0.1, 0.5, chrono.ChVectorD(0, -0.2, 0))  # Simplified collision
terrain.SetCollisionModel(terrain_collision)
terrain.SetCollide(True)

# Add texture to terrain
vis_mat = chrono.ChVisualMaterial()
vis_mat.SetKdTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
terrain_shape.AddMaterial(vis_mat)
system.Add(terrain)

# Initialize Curiosity rover
rover = curiosity.Curiosity(system, chrono.ChContactMethod_NSC)
rover.Initialize(chrono.ChFrameD(chrono.ChVectorD(0, 0.2, 0), chrono.QUNIT))
rover.SetTireCollisionType(veh.ChTire.CollisionType_SINGLE_POINT)

# Create Irrlicht visualization
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Curiosity Rover Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(3, 3, 3), chrono.ChVectorD(0, 0, 0))
vis.AddLightWithShadow(chrono.ChVectorD(5, 8, -5), chrono.ChVectorD(0, 0, 0), 50, 5, 50, 35, 512)

# Custom driver for rover control
class RoverDriver(chrono.ChDriver):
    def __init__(self, rover):
        super().__init__(*rover.GetChassisBody())
        self.speed = 0.0
        self.steering = 0.0
        
    def ProcessInputs(self, time):
        self.SetThrottle(min(max(self.speed, -1.0), 1.0))
        self.SetSteering(min(max(self.steering, -1.0), 1.0))

driver = RoverDriver(rover)
rover.SetDriver(driver)

# Simulation loop parameters
time_step = 0.005
realtime_step = True
frame = 0

while vis.Run():
    # Handle controls
    if vis.GetDevice().KeyDown(irr.KEY_KEY_W):
        driver.speed += 0.1 * time_step
    elif vis.GetDevice().KeyDown(irr.KEY_KEY_S):
        driver.speed -= 0.1 * time_step
    if vis.GetDevice().KeyDown(irr.KEY_KEY_A):
        driver.steering += 0.5 * time_step
    elif vis.GetDevice().KeyDown(irr.KEY_KEY_D):
        driver.steering -= 0.5 * time_step

    # Update visualization and simulation
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    # Update rover systems
    driver.ProcessInputs(system.GetChTime())
    rover.Update()
    
    # Advance simulation
    system.DoStepDynamics(time_step)
    
    # Update camera position
    if frame % 10 == 0:
        chassis_pos = rover.GetChassis().GetPos()
        vis.GetActiveCamera().setTarget(irr.vector3df(chassis_pos.x, chassis_pos.y, chassis_pos.z))
    
    frame += 1