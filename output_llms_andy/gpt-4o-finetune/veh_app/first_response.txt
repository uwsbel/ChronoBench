import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math

""" 
Chrono simulation code setup. 
"""
# Set the Chrono data directory
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initialize the physical simulation system
sys = chrono.ChSystemNSC()

# Initialize the visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Vehicle Demo: Gator')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 34, 0.2), chrono.ChVector3d(0, 0, 0.2))
vis.AddTypicalLights()
vis.SetLightIntensity(1.0)

# Add a point light to the scene
light_pos = chrono.ChVector3f(2.0, 4.0, 2.0)
vis.GetSceneManager().addLightSource(None, light_pos, chrono.ChColor(1, 1, 1), 0.8)

# Create the Gator vehicle and add it to the simulation system
init_pos = chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.6), chrono.QuatFromAngleZ(0))
vehicle = veh.Gator()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(init_pos)
vehicle.SetTireType(veh.TireType.RIGID)
vehicle.SetTireStepSize(timestep)
vehicle.Initialize()

# Set visualization types for various vehicle parts
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

# Add a point light to the sensor manager
point_light = sens.ChLightManager().AddPointLight(intensity=1.0, color=chrono.ChColor(1, 1, 1),
                                                   abs_coords=light_pos, specular_coefficient=2)

# Create the terrain for the vehicle to interact with
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(sys)
patch = terrain.AddPatch(patch_mat, 
                         chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                         100.0, 100.0)

patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 100, 100)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()
# Create and initialize the interactive driver system
driver = veh.ChInteractiveDriverIRR(vis)

# Set the target speed for the vehicle
driver.SetSpeed(6.0)

# Initialize the sensor manager
manager = sens.ChSensorManager(sys)
# Add point lights to the scene
light_list = vis.GetSceneManager().getLights()
for light in light_list:
    mgr_lgt = manager.AddLight(RemoveFromScene=False)
    mgr_lgt.SetIntensity(light.Intensity)
    mgr_lgt.SetLightColor(chrono.ChColor(light.Color.r, light.Color.g, light.Color.b))
    mgr_lgt.SetCoords(chrono.ChVector3f(light.Position.x, light.Position.y, light.Position.z))
    mgr_lgt.EnableCastShadows(light.CastShadows)
mgr_lgt = manager.AddLight(sens.ChLightData())
mgr_lgt.SetType(chrono.guid.SpatialApiPointLight)

# Create a camera sensor and attach it to the vehicle's chassis
cam = sens.ChCameraSensor(
    vehicle.GetChassisBody(),
    update_rate,
    sens.chrono.ChFrustum(chrono.ChRectangle(-1, 1, -0.75, 0.75), 1),  # rview camera
    vis_width,
    vis_height,
    vis_filter)
cam.SetName("Gator POV Camera")
cam.SetLag(0)
cam.SetOffsetPose(chrono.ChFramed(chrono.ChVector3d(2, 0, 1.4), chrono.QuatFromAngleAxis(chrono.CH_PI_4, chrono.VECT_Y)))

# Register the camera filter graph
if vis:
    cam.PushFilter(sens.ChFilterVisualize(vis_width, vis_height, "Gator POV"))
cam.PushFilter(sens.ChFilterRGBA8Access())
manager.AddSensor(cam)

# ---------------
# Simulation loop
# ---------------
# Initialize the simulation frame counter and timing variables
frame = 0
time = 0

# Set the simulation's realtime step
ts = vehicle.GetSystem().GetStepSize()

# Initialize the realtime step timer
realtime_timer = chrono.ChRealtimeStepTimer()

while vis.Run():
    time = vehicle.GetChassisBody().GetPos().Length()

    # Retrieve driver inputs and advance the simulation modules
    driver_inputs = driver.GetInputs()
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    manager.Update()
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    driver.Advance(ts)
    terrain.Advance(ts)
    vehicle.Advance(ts)
    # Increment the simulation frame counter
    frame += 1

    # Advance the simulation by one timestep
    sys.DoStepDynamics(ts)

    # Ensure the simulation runs in real-time
    realtime_timer.Spin(ts)