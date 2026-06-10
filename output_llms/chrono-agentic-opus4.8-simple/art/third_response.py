import math                                                           # render-cadence math
import pychrono.core as chrono                                        # core types
import pychrono.vehicle as veh                                        # vehicle catalog (ARTcar)
import pychrono.irrlicht as chronoirr                                 # Irrlicht render

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

init_loc = chrono.ChVector3d(0, 0, 0.2)                              # spawn near terrain top
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                          # identity orientation
step_size = 1e-3                                                      # integration step
tire_step_size = 1e-3                                                 # tire model step

car = veh.ARTcar()                                                   # ARTcar RC vehicle wrapper
car.SetContactMethod(chrono.ChContactMethod_NSC)                     # NSC for rigid terrain
car.SetChassisCollisionType(veh.CollisionType_NONE)                 # no chassis collision mesh
car.SetChassisFixed(False)                                           # MANDATORY — chassis must move
car.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))         # initial pose
car.SetTireType(veh.TireModelType_RIGID)                            # rigid tire on rigid terrain
car.SetTireStepSize(tire_step_size)                                 # tire integration step
car.SetMaxMotorVoltageRatio(0.26)                                  # faster: motor voltage ratio
car.SetStallTorque(0.4)                                            # faster: higher stall torque
car.SetTireRollingResistance(0.03)                                # faster: lower rolling resistance
car.Initialize()                                                    # build the vehicle

car.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)  # chassis primitives
car.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)  # suspension primitives
car.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)    # steering primitives
car.SetWheelVisualizationType(veh.VisualizationType_MESH)         # wheel mesh
car.SetTireVisualizationType(veh.VisualizationType_MESH)          # tire mesh

system = car.GetSystem()                                            # take wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact
print("VEHICLE MASS: ", car.GetVehicle().GetMass())                 # report total vehicle mass

terrain = veh.RigidTerrain(system)                                  # rigid flat terrain
patch_mat = chrono.ChContactMaterialNSC()                           # NSC patch material
patch_mat.SetFriction(0.9)                                          # terrain friction
patch_mat.SetRestitution(0.01)                                      # terrain restitution
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 100.0, 100.0)  # flat 100x100 m patch
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                       # patch color
terrain.Initialize()                                               # build terrain

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                   # vehicle Irrlicht vis
vis.SetWindowTitle("ARTcar on Rigid Terrain")                     # window title
vis.SetWindowSize(1280, 1024)                                     # window size
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 0.3), 3.0, 0.5)       # chase camera on the car
vis.Initialize()                                                  # build device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png")) # logo
vis.AddSkyBox()                                                   # sky box
vis.AddLightDirectional()                                        # directional light
vis.AttachVehicle(car.GetVehicle())                              # bind vehicle visuals

render_step_size = 1.0 / 50.0                                     # 50 fps render cadence
render_steps = math.ceil(render_step_size / step_size)           # physics steps per frame

driver = veh.ChInteractiveDriverIRR(vis)                         # interactive keyboard driver
driver.SetSteeringDelta(render_step_size / 1.0)                  # steering ramp rate
driver.SetThrottleDelta(render_step_size / 1.0)                  # throttle ramp rate
driver.SetBrakingDelta(render_step_size / 0.3)                  # braking ramp rate
driver.Initialize()                                             # build driver


realtime_timer = chrono.ChRealtimeStepTimer()                   # wall-clock pacing
step_number = 0                                                 # physics step counter
while vis.Run():
    time = system.GetChTime()                                  # current sim time

    if step_number % render_steps == 0:                        # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()


    driver_inputs = driver.GetInputs()                         # current driver inputs

    driver.Synchronize(time)                                   # update driver
    terrain.Synchronize(time)                                  # update terrain
    car.Synchronize(time, driver_inputs, terrain)             # update vehicle with inputs
    vis.Synchronize(time, driver_inputs)                      # update visuals

    driver.Advance(step_size)                                  # advance driver
    terrain.Advance(step_size)                                 # advance terrain
    car.Advance(step_size)                                     # advances wrapper-owned system
    vis.Advance(step_size)                                     # advance visuals

    step_number += 1                                           # next step
    realtime_timer.Spin(step_size)                             # spin to match wall clock
