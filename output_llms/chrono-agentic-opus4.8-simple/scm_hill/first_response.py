import math                                                            # render-cadence math
import pychrono.core as chrono                                         # core PyChrono
import pychrono.vehicle as veh                                         # vehicle module
import pychrono.irrlicht as chronoirr                                  # Irrlicht visualization

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                  # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

step_size = 2e-3                                                       # integration step (SCM needs small)
tire_step_size = 1e-3                                                  # tire force model step

init_loc = chrono.ChVector3d(-5, 0, 0.6)                              # initial chassis position
init_rot = chrono.QuatFromAngleZ(0)                                   # facing +X, no yaw

hmmwv = veh.HMMWV_Full()                                               # full HMMWV catalog model
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)                    # SMC for SCM deformable terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)                 # no chassis collision box
hmmwv.SetChassisFixed(False)                                          # chassis must be free to move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))        # initial pose
hmmwv.SetTireType(veh.TireModelType_TMEASY)                          # TMEASY tire (RIGID won't drive on SCM)
hmmwv.SetTireStepSize(tire_step_size)                                # tire integration step
hmmwv.Initialize()                                                    # build the vehicle

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)     # chassis mesh
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)  # suspension primitives
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)    # steering primitives
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)       # wheel mesh
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)        # tire mesh

system = hmmwv.GetSystem()                                            # take the wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED, after Initialize, before SCM
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())               # report total vehicle mass

terrain = veh.SCMTerrain(system)                                      # Bekker-Wong deformable terrain
terrain.SetSoilParameters(
    2e6,    # Bekker_Kphi    — frictional modulus (Pa)
    0,      # Bekker_Kc      — cohesive modulus
    1.1,    # Bekker_n       — exponent
    0,      # Mohr_cohesion  — cohesive limit (Pa)
    30,     # Mohr_friction  — friction angle (deg)
    0.01,   # Janosi_shear   — shear coefficient (m)
    2e8,    # elastic_K      — elastic stiffness (Pa/m)
    3e4,    # damping_R      — vertical damping (Pa·s/m)
)
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.1)          # colored sinkage heatmap
terrain.AddMovingPatch(
    hmmwv.GetChassisBody(),                                           # chassis body — stable OOBB
    chrono.ChVector3d(0, 0, 0),                                      # local OOBB centre offset
    chrono.ChVector3d(5, 3, 1),                                      # OOBB dimensions (m)
)
terrain.Initialize(veh.GetDataFile("terrain/height_maps/bump64.bmp"),  # bump hill heightmap
                   40, 40, -1, 1, 0.02)                              # length, width, hMin, hMax, resolution
terrain.SetMeshWireframe(False)                                      # solid mesh, not wireframe
terrain.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)  # dirt texture

tire_rad = hmmwv.GetVehicle().GetAxles()[0].m_wheels[0].GetTire().GetRadius()  # tire radius
tire_w = hmmwv.GetVehicle().GetAxles()[0].m_wheels[0].GetTire().GetWidth()     # tire width
tire_mat = chrono.ChContactMaterialSMC()                            # SMC tire material
tire_mat.SetFriction(0.9)                                           # tire friction
tire_mat.SetRestitution(0.1)                                        # tire restitution

TIRE_FAMILY = 1                                                      # collision family for tire cylinders
for axle in hmmwv.GetVehicle().GetAxles():                          # add a collision cylinder per spindle
    for iw in range(2):                                             # left and right wheel
        spindle = axle.m_wheels[iw].GetSpindle()                    # the spindle body
        spindle.AddCollisionShape(
            chrono.ChCollisionShapeCylinder(tire_mat, tire_rad + 0.04, tire_w),  # +0.04 so SCM sees sinkage
            chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(math.pi / 2)),    # align cylinder axis
        )
        spindle.EnableCollision(True)                              # enable spindle collision
        sp_cm = spindle.GetCollisionModel()                       # collision model
        sp_cm.SetFamily(TIRE_FAMILY)                              # tag the family
        sp_cm.DisallowCollisionsWith(TIRE_FAMILY)                # tires don't collide with each other
system.GetCollisionSystem().BindAll()                             # rebuild collision models after edits

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                  # vehicle-specific Irrlicht system
vis.SetWindowTitle("HMMWV on SCM Hill")                           # window title
vis.SetWindowSize(1280, 1024)                                     # window size
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 6.0, 0.5)      # chase camera behind chassis
vis.Initialize()                                                  # build the device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png")) # PyChrono logo
vis.AddSkyBox()                                                   # sky box
vis.AddLightDirectional()                                         # directional light (vehicle truth)
vis.AttachVehicle(hmmwv.GetVehicle())                            # bind vehicle visual assets

render_step_size = 1.0 / 50.0                                     # render cadence (50 FPS)
render_steps = math.ceil(render_step_size / step_size)           # physics steps per rendered frame

driver = veh.ChInteractiveDriverIRR(vis)                         # interactive keyboard driver
steering_time = 1.0                                              # s to reach full steering
throttle_time = 1.0                                              # s to reach full throttle
braking_time = 0.3                                               # s to reach full braking
driver.SetSteeringDelta(render_step_size / steering_time)        # steering rate
driver.SetThrottleDelta(render_step_size / throttle_time)        # throttle rate
driver.SetBrakingDelta(render_step_size / braking_time)          # braking rate
driver.Initialize()                                             # initialize the driver

render_every = render_steps                                      # untagged cadence constant
sim_end = 12.0                                                   # simulation duration (s)

realtime_timer = chrono.ChRealtimeStepTimer()                  # wall-clock pacing
while vis.Run() and system.GetChTime() < sim_end:             # main real-time loop
    vis.BeginScene()                                          # begin frame
    vis.Render()                                              # render scene
    vis.EndScene()                                            # end frame
    for _ in range(render_every):                            # advance physics in a batch
        time = system.GetChTime()                            # current sim time
        driver_inputs = driver.GetInputs()                   # current driver inputs
        driver.Synchronize(time)                             # sync driver
        terrain.Synchronize(time)                            # sync terrain
        hmmwv.Synchronize(time, driver_inputs, terrain)      # sync vehicle with terrain
        vis.Synchronize(time, driver_inputs)                 # sync visualization
        driver.Advance(step_size)                            # advance driver
        terrain.Advance(step_size)                           # advance terrain
        hmmwv.Advance(step_size)                             # advance vehicle (steps the system)
        vis.Advance(step_size)                               # advance visualization
        realtime_timer.Spin(step_size)                       # pace to wall clock
        if system.GetChTime() >= sim_end:                    # stop at sim_end
            break
