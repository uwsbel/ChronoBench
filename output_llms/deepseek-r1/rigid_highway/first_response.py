import os
import math
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
from pychrono import ChVectorD as ChV

# Initialize PyChrono environment
chrono.SetChronoDataPath(os.path.join(os.environ['CHRONO_DATA_DIR'], 'data/'))
system = chrono.ChSystemNSC()
system.Set_G_acc(ChV(0, 0, -9.81))

# Create Irrlicht visualization
application = irr.ChIrrApp(system, "HMMWV Simulation", irr.dimension2d(1280,720))
application.AddTypicalLights()
application.AddTypicalCamera(irr.vector3df(0,6,1))

# Create and configure terrain
terrain = chrono.ChBody()
terrain.SetPos(ChV(0, 0, 0))
terrain.SetBodyFixed(True)

# Collision and visualization meshes for terrain
collision_mesh = chrono.ChTriangleMeshConnected()
collision_mesh.LoadWavefrontMesh(chrono.GetChronoDataFile("highway_col.obj"))
terrain_shape = chrono.ChTriangleMeshShape()
terrain_shape.SetMesh(collision_mesh)
terrain.GetAssets().push_back(terrain_shape)

visual_mesh = chrono.ChObjFileShape()
visual_mesh.SetFilename(chrono.GetChronoDataFile("highway_vis.obj"))
terrain.GetAssets().push_back(visual_mesh)

system.Add(terrain)

# Initialize HMMWV vehicle
hmmwv = veh.HMMWV_Full(system)
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysD(ChV(0, 0, 0.5), chrono.Q_from_AngZ(0)))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)

hmmwv.Initialize()

# Set tire parameters
tire_pressure = 35e3
for axle in [veh.LEFT, veh.RIGHT]:
    hmmwv.SetTirePressure(tire_pressure, axle, veh.FRONT)
    hmmwv.SetTirePressure(tire_pressure, axle, veh.REAR)
    hmmwv.SetTireDamping(tire_pressure/50, axle, veh.FRONT)
    hmmwv.SetTireDamping(tire_pressure/50, axle, veh.REAR)

# Create interactive driver
driver = veh.ChIrrGuiDriver(application)
steering_time = 1.0
driver.SetSteeringDelta(1.0 / (50 * steering_time))
driver.SetThrottleDelta(1.0 / 5.0)
driver.SetBrakingDelta(1.0 / 5.0)

# Finalize Irrlicht setup
application.AssetBindAll()
application.AssetUpdateAll()
driver.Initialize()

# Simulation parameters
step_size = 0.02
render_step = 1.0 / 50  # 50 FPS
realtime_timer = chrono.ChRealtimeStepTimer()

# Main simulation loop
time = 0.0
while application.GetDevice().run():
    time = system.GetChTime()
    
    # Update driver inputs
    driver_inputs = driver.GetInputs()
    hmmwv.Synchronize(time, driver_inputs, terrain)
    
    # Advance simulation
    application.BeginScene(True, True, irr.SColor(255,140,161,192))
    application.DrawAll()
    application.DoStepDynamics(step_size)
    application.EndScene()
    
    # Maintain real-time simulation
    realtime_timer.Spin(step_size)

# Cleanup
application.GetDevice().drop()