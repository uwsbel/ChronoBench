import os
import math
import numpy as np
import pychrono
import pychrono.robot
import pychrono.irrlicht
import pychrono.vehicle

# Initialize PyChrono
chrono.SetChronoDataPath(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Create the system
sys = chrono.ChSystemSMC()

# Initialize the vehicle
gator = veh.Gator(sys)
gator.SetContactMethod(chrono.ChContactMaterialSMC())
gator.SetChassisCollisionType(chrono.CollisionType.CHASSIS_FRICTION)
gator.Initialize(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI)))

# Initialize the terrain
terrain = veh.RigidTerrain(gator.GetSystem())
# Add some flat patches for realism
patch = terrain.AddPatch(
    patch_mat=chrono.ChMaterial(),
    position=chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    length=terrainLength,
    width=terrainWidth
)
terrain.Initialize()

# Initialize the driver
driver = robosimian.RoBoSimianDriver(sys, True, True)
driver.Initialize(gator)

# Setup visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Gator Vehicle Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('pychrono.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 5, 10))
vis.AddTypicalLights()

# Setup camera
camera = vis.AddCamera(chrono.ChVector3d(0, 5, 10))
camera.SetLookAt(gator.GetChassis().GetEndEffector().GetPosition())
camera.SetFieldOfView(math.radians(45))

# Setup lights
light1 = vis.AddLight(chrono.ChVector3d(5, 5, 5))
light2 = vis.AddLight(chrono.ChVector3d(-5, 5, 5))
light3 = vis.AddLight(chrono.ChVector3d(0, 5, 10))

# Setup sensor manager
sensor_manager = sm.SensorManager()
sensor_manager.AddPointLight(light1, 10.0, 10.0, 10.0)
sensor_manager.AddCamera(camera, 512, 512)
gator.GetChassis().AddSensorManager(sensor_manager)
gator.GetChassis().AddCameraSensor(camera)

# Simulation loop
while vis.Run():
    # Update driver
    driver.Update()
    
    # Update terrain
    terrain.Update()
    
    # Update vehicle
    gator.Update()
    
    # Update contact points
    sys.DoStepDynamics(0.01)
    
    # Render
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    # Handle camera input
    key = vis.GetKey()
    if key:
        camera.Control(key)