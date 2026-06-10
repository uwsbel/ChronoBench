import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

# initial position / orientation
init_loc = chrono.ChVector3d(0, 0, 0.5)                             # start on flat ground
init_rot = chrono.QuatFromAngleZ(0)                                  # facing +X

step_size = 1e-3                                                     # simulation time step
sim_end = 4.0                                                        # end at 4 seconds
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * step_size)))        # cadence constant

hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)                   # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                                         # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
hmmwv.SetTireType(veh.TireModelType_TMEASY)                         # TMEASY for good traction
hmmwv.SetTireStepSize(step_size)
hmmwv.Initialize()

system = hmmwv.GetSystem()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())               # report total vehicle mass

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)       # veh namespace for vehicle vis types
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# rigid terrain
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200.0, 200.0)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# vehicle Irrlicht visualization
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV Custom Driver")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)        # chase camera behind chassis
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                                             # vehicle demos use directional light
vis.AttachVehicle(hmmwv.GetVehicle())

# custom driver class with delay parameter of 0.5
class MyDriver(veh.ChDriver):
    def __init__(self, vehicle, delay=0.5):
        super().__init__(vehicle)
        self.delay = delay                                            # delay before inputs are applied

    def Synchronize(self, time):
        t = time - self.delay                                         # effective time after delay
        if t < 0:
            self.SetThrottle(0.0)
            self.SetSteering(0.0)
            self.SetBraking(0.0)
        else:
            # throttle gradually increases to 0.7 after 0.2 s of effective time
            if t < 0.2:
                throttle = 0.7 * (t / 0.2)                           # ramp from 0 to 0.7
            else:
                throttle = 0.7                                        # hold at 0.7
            self.SetThrottle(throttle)
            self.SetBraking(0.0)
            # steering: sinusoidal pattern starting at 2 s (effective time)
            if t >= 2.0:
                self.SetSteering(math.sin(t - 2.0))                  # sinusoidal steering after 2 s
            else:
                self.SetSteering(0.0)

driver = MyDriver(hmmwv.GetVehicle(), delay=0.5)                     # init with delay=0.5
driver.Initialize()

realtime_timer = chrono.ChRealtimeStepTimer()

while vis.Run() and system.GetChTime() < sim_end:
    sim_time = system.GetChTime()

    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    for _ in range(render_every):
        sim_time = system.GetChTime()
        if sim_time >= sim_end:                                       # end condition: 4 seconds
            break
        driver_inputs = driver.GetInputs()

        driver.Synchronize(sim_time)
        terrain.Synchronize(sim_time)
        hmmwv.Synchronize(sim_time, driver_inputs, terrain)
        vis.Synchronize(sim_time, driver_inputs)

        driver.Advance(step_size)
        terrain.Advance(step_size)
        hmmwv.Advance(step_size)                                      # advances wrapper-owned system
        vis.Advance(step_size)

        realtime_timer.Spin(step_size)                               # real-time pacing
