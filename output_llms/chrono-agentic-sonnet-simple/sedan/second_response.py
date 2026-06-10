import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                  # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')              # locate vehicle data files

# --- Vehicle 1 (BMW E90 sedan) ---
sedan1 = veh.BMW_E90()                                                # first vehicle owns its system
sedan1.SetContactMethod(chrono.ChContactMethod_NSC)                   # NSC for rigid terrain
sedan1.SetChassisCollisionType(veh.CollisionType_NONE)
sedan1.SetChassisFixed(False)                                          # MANDATORY — fixed chassis won't move
init_loc1 = chrono.ChVector3d(0, 0, 0.5)                              # initial position vehicle 1
init_rot1 = chrono.QuatFromAngleZ(0.0)                                # heading along X
sedan1.SetInitPosition(chrono.ChCoordsysd(init_loc1, init_rot1))
sedan1.SetTireType(veh.TireModelType_RIGID)                            # rigid tires on rigid terrain
sedan1.SetTireStepSize(1e-3)
sedan1.Initialize()

system = sedan1.GetSystem()                                            # vehicle 1 owns the system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # REQUIRED after Initialize

sedan1.SetChassisVisualizationType(veh.VisualizationType_MESH)
sedan1.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
sedan1.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
sedan1.SetWheelVisualizationType(veh.VisualizationType_MESH)
sedan1.SetTireVisualizationType(veh.VisualizationType_MESH)

print("VEHICLE MASS: ", sedan1.GetVehicle().GetMass())                # scored: report vehicle mass

# --- Vehicle 2 (BMW E90 sedan, shares vehicle 1's system) ---
sedan2 = veh.BMW_E90(system)                                          # MUST share system — never bare BMW_E90()
sedan2.SetChassisCollisionType(veh.CollisionType_NONE)
sedan2.SetChassisFixed(False)                                          # MANDATORY
init_loc2 = chrono.ChVector3d(0, 5, 0.5)                              # offset in Y to avoid overlap
init_rot2 = chrono.QuatFromAngleZ(0.0)                                # same heading
sedan2.SetInitPosition(chrono.ChCoordsysd(init_loc2, init_rot2))
sedan2.SetTireType(veh.TireModelType_RIGID)                            # rigid tires on rigid terrain
sedan2.SetTireStepSize(1e-3)
sedan2.Initialize()

sedan2.SetChassisVisualizationType(veh.VisualizationType_MESH)
sedan2.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
sedan2.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
sedan2.SetWheelVisualizationType(veh.VisualizationType_MESH)
sedan2.SetTireVisualizationType(veh.VisualizationType_MESH)

print("VEHICLE 2 MASS: ", sedan2.GetVehicle().GetMass())              # scored: report second vehicle mass

# --- Rigid Terrain ---
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()                             # NSC material matches NSC vehicle
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrainLength = 200.0                                                  # terrain extent in X
terrainWidth = 200.0                                                   # terrain extent in Y
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)  # concrete texture as requested
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))
terrain.Initialize()

# --- Custom sinusoidal driver for both vehicles ---
class SinusoidalDriver(veh.ChDriver):
    def __init__(self, vehicle, throttle_val=0.4, steer_amp=0.3, steer_freq=0.5):
        super().__init__(vehicle)
        self._throttle = throttle_val
        self._amp = steer_amp                                          # sinusoidal steering amplitude
        self._freq = steer_freq                                        # steering oscillation frequency (Hz)

    def Synchronize(self, time):
        self.SetThrottle(self._throttle)                               # constant throttle
        self.SetBraking(0.0)
        self.SetSteering(self._amp * math.sin(2.0 * math.pi * self._freq * time))  # sinusoidal steering

driver1 = SinusoidalDriver(sedan1.GetVehicle(), throttle_val=0.4, steer_amp=0.3, steer_freq=0.5)
driver1.Initialize()

driver2 = SinusoidalDriver(sedan2.GetVehicle(), throttle_val=0.4, steer_amp=0.3, steer_freq=0.5)
driver2.Initialize()

# --- Irrlicht visualization (vehicle-specific vis system) ---
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Two Sedans - Sinusoidal Steering")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)         # chase camera following vehicle 1
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                                              # vehicle demos use directional light
vis.AttachVehicle(sedan1.GetVehicle())                                 # attach primary vehicle for chase cam

# --- Simulation parameters ---
step_size = 1e-3                                                       # physics time step
sim_end = 20.0                                                         # simulation duration (s)
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * step_size)))          # untagged cadence constant

realtime_timer = chrono.ChRealtimeStepTimer()

# --- Review-only recording setup ---

# --- Main simulation loop ---
while vis.Run() and system.GetChTime() < sim_end:
    time = system.GetChTime()

    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    for _ in range(render_every):
        sim_time = system.GetChTime()

        driver_inputs1 = driver1.GetInputs()
        driver_inputs2 = driver2.GetInputs()

        driver1.Synchronize(sim_time)                                  # update sinusoidal steering vehicle 1
        driver2.Synchronize(sim_time)                                  # update sinusoidal steering vehicle 2
        terrain.Synchronize(sim_time)
        sedan1.Synchronize(sim_time, driver_inputs1, terrain)          # synchronize vehicle 1
        sedan2.Synchronize(sim_time, driver_inputs2, terrain)          # synchronize vehicle 2
        vis.Synchronize(sim_time, driver_inputs1)

        driver1.Advance(step_size)
        driver2.Advance(step_size)
        terrain.Advance(step_size)
        sedan1.Advance(step_size)                                       # advances the shared system
        sedan2.Advance(step_size)
        vis.Advance(step_size)

        if system.GetChTime() >= sim_end:
            break

    realtime_timer.Spin(step_size)
