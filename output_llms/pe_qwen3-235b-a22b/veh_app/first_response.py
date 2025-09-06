import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens
import numpy as np

# Set Chrono data path
chrono.SetChronoDataPath('PYCHRONO_DATA_DIR')

# Create the physical system
sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Gator Vehicle Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 6))
vis.AddTypicalLights()

# Create Gator vehicle
gator = veh.Gator(sys)
gator.SetContactMethod(chrono.ChContactMethod_NSC)
gator.SetChassisCollisionType(veh.Gator.ChassisCollisionType_BOX)
gator.SetChassisFixed(False)
gator.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.5), chrono.QUNIT))
gator.Initialize()

# Customize vehicle visualization
chassis = gator.GetChassisBody()
chassis_shape = chrono.ChVisualShapeBox(chrono.ChVector3d(1.5, 0.8, 3.0))
chassis_shape.SetColor(chrono.ChColor(0.8, 0.2, 0.2))  # Red chassis
chassis.AddVisualShape(chassis_shape)

# Create rigid terrain
terrain = veh.RigidTerrain(sys)
patch = terrain.AddPatch(chrono.ChMaterialSurfaceNSC(), 
                        chrono.ChCoordsysd(), 
                        100, 100)  # 100x100 terrain
patch.SetColor(chrono.ChColor(0.3, 0.6, 0.3))  # Green terrain
terrain.Initialize()

# Create driver system
driver = veh.ChPathFollowerDriver(gator, 
                                 veh.ChPath(),  # Empty path for manual control
                                 0.1,           # Lookahead distance
                                 10)            # Target speed
driver.Initialize()

# Create sensor manager
manager = sens.ChSensorManager(sys)

# Add point lights
manager.scene.AddPointLight(chrono.ChVector3f(10, 10, 10), chrono.ChColor(1, 1, 1), 1000)
manager.scene.AddPointLight(chrono.ChVector3f(-10, 10, 10), chrono.ChColor(1, 1, 1), 1000)

# Add camera to chassis
camera = sens.ChCameraSensor(
    gator.GetChassisBody(),  # Parent body
    60.0,  # Update rate in Hz
    chrono.ChFrameD(chrono.ChVector3d(0, 1.5, 1.0),  # Position relative to chassis
                   chrono.Q_from_AngAxis(chrono.CH_PI_2, chrono.ChVector3d(0, 1, 0))),  # Orientation
    1280,  # Width
    720,   # Height
    chrono.CH_C_PI / 3  # Field of view
)
camera.SetName("Gator Camera")
manager.AddSensor(camera)

# Simulation loop
time_step = 1e-3
end_time = 10.0

while vis.Run() and sys.GetChTime() < end_time:
    # Synchronize modules
    driver.Synchronize(sys.GetChTime())
    terrain.Synchronize(sys.GetChTime())
    gator.Synchronize(sys.GetChTime(), driver.GetInputs(), terrain)
    manager.Update()
    
    # Advance simulation
    sys.DoStepDynamics(time_step)
    
    # Render visualization
    vis.BeginScene()
    vis.Render()
    vis.EndScene()