import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set paths
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Simulation parameters
initLoc = chrono.ChVectorD(0, 0, 0.5)
initLoc2 = chrono.ChVectorD(5.0, 0.0, 0.5)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)
vis_type = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE
tire_model = veh.TireModelType_TMEASY
step_size = 1e-3
render_step_size = 1.0 / 50

# Create shared system
sys = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create first vehicle
vehicle = veh.Sedan()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(step_size)
vehicle.Initialize(sys)
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# Create second vehicle
vehicle2 = veh.Sedan()
vehicle2.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle2.SetChassisCollisionType(chassis_collision_type)
vehicle2.SetChassisFixed(False)
vehicle2.SetInitPosition(chrono.ChCoordsysD(initLoc2, initRot))
vehicle2.SetTireType(tire_model)
vehicle2.SetTireStepSize(step_size)
vehicle2.Initialize(sys)
vehicle2.SetChassisVisualizationType(vis_type)
vehicle2.SetSuspensionVisualizationType(vis_type)
vehicle2.SetSteeringVisualizationType(vis_type)
vehicle2.SetWheelVisualizationType(vis_type)
vehicle2.SetTireVisualizationType(vis_type)

# Create terrain with concrete texture
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(sys)
patch = terrain.AddPatch(patch_mat,
                        chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
                        100.0, 100.0)
patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Visualization setup
vis = irr.ChIrrApp(sys, "Dual Sedan Simulation", irr.dimension2du(1280, 1024))
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.SetChaseCamera(chrono.ChVectorD(-5, 0, 1.8), 6.0, 0.5)
vis.SetTimestep(step_size)
vis.AssetBind()
vis.AssetUpdate()
vis.AddCamera(chrono.ChVectorD(0, 3, 10))
vis.SetVideoframeSave(True)
vis.SetVideoframeRate(5)

# Attach both vehicles to visual system
vis.Add(vehicle.GetVehicle())
vis.Add(vehicle2.GetVehicle())

# Drivers for both vehicles
driver = veh.ChIrrGuiDriver(vis)
driver.SetSteeringDelta(render_step_size / 1.0)
driver.SetThrottleDelta(render_step_size / 1.0)
driver.SetBrakingDelta(render_step_size / 0.3)
driver.Initialize()

driver2 = veh.ChIrrGuiDriver(vis)
driver2.SetSteeringDelta(render_step_size / 1.0)
driver2.SetThrottleDelta(render_step_size / 1.0)
driver2.SetBrakingDelta(render_step_size / 0.3)
driver2.Initialize()

# Simulation loop
render_steps = int(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()

step_number = 0
while vis.Run():
    time = sys.GetChTime()
    
    # Update sinusoidal steering for both vehicles
    steering = math.sin(time * 2 * math.pi * 0.5) * 0.5  # 0.5 Hz, amplitude 0.5 radians
    driver.SetSteering(steering)
    driver2.SetSteering(steering)
    
    # Process user inputs
    driver.Synchronize(time)
    driver2.Synchronize(time)
    
    # Update vehicle states
    vehicle.Synchronize(time, driver.GetInputs(), terrain)
    vehicle2.Synchronize(time, driver2.GetInputs(), terrain)
    
    # Advance simulation
    sys.DoStepDynamics(step_size)
    
    # Render
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
    
    vis Advance()
    step_number += 1
    realtime_timer.Spin(step_size)