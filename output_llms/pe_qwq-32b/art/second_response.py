import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set data paths correctly
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation (modified to (1,0,0.5))
initLoc = chrono.ChVectorD(1, 0, 0.5)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)

# Visualization type changed to PRIMITIVES
vis_type = veh.VisualizationType_PRIMITIVES

# Chassis collision type changed to MESH
chassis_collision_type = veh.CollisionType_MESH

# Tire model type changed to FIALA
tire_model = veh.TireModelType_FIALA

terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0
trackPoint = chrono.ChVectorD(0.0, 0.0, 0.2)
contact_method = chrono.ChContactMethod_NSC
contact_vis = False
step_size = 1e-3
tire_step_size = step_size
render_step_size = 1.0 / 50

# Create the ARTcar vehicle
vehicle = veh.ARTcar()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.SetMaxMotorVoltageRatio(0.16)
vehicle.SetStallTorque(0.3)
vehicle.SetTireRollingResistance(0.06)

vehicle.Initialize()

# Set visualization types
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# Terrain setup
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
    terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Visualization interface
vis = irr.ChIrrApp(vehicle.GetSystem(), 'dart', irr.dimension2du(1280, 1024))
vis.SetCameraPosition(chrono.ChVectorD(1, 0, 2), chrono.ChVectorD(1, 0, 0))
vis.AddTypicalLights()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.SetTimestep(step_size)
vis.SetVideoframeSave(True)
vis.SetVideoframeRate(10)
vis.SetVideoframeFilename("output/frame_%05d.jpg")

# Attach vehicle to visualization
vis.AttachVehicle(vehicle)

# Initialize visualization
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.GetSystem().SetBodyCollideable(vehicle.GetChassis(), True)

# Driver setup
driver = veh.ChIrrGuiDriver(vis)
driver.SetSteeringDelta(render_step_size / 1.0)
driver.SetThrottleDelta(render_step_size / 1.0)
driver.SetBrakingDelta(render_step_size / 0.3)
driver.Initialize()

# Simulation loop
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

print("VEHICLE MASS: ", vehicle.GetMass())

while vis.Run():
    time = vehicle.GetSystem().GetChTime()
    
    # Render scene
    if step_number % math.ceil(render_step_size / step_size) == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
    
    driver_inputs = driver.GetInputs()
    
    # Synchronize modules
    driver.Synchronize(time, driver_inputs)
    vehicle.Synchronize(time, driver_inputs)
    terrain.Synchronize(time)
    
    # Advance simulation
    driver.Advance(step_size)
    vehicle.Advance(step_size)
    terrain.Advance(step_size)
    
    step_number += 1
    realtime_timer.Spin(step_size)