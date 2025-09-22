#
#  Two–vehicle demo – both cars run on the same rigid concrete terrain.
#  The first Sedan is driven from the keyboard, but its steering wheel
#  is continuously “helped” by a sinusoid.  
#  The second Sedan is controlled entirely by an automatic (sinusoidal)
#  driver implemented below.
#

import math
import pychrono.core    as chrono
import pychrono.irrlicht as irr   # required by the interactive driver
import pychrono.vehicle as veh


# -------------------------------------------------------------------------
# 1.  Chrono data directories
# -------------------------------------------------------------------------
chrono.SetChronoDataPath( chrono.GetChronoDataPath() )
veh.SetDataPath        ( chrono.GetChronoDataPath() + 'vehicle/' )


# -------------------------------------------------------------------------
# 2.  General set-up
# -------------------------------------------------------------------------
initLoc1 = chrono.ChVector3d(0.0,  0.0, 0.5)     # 1st vehicle
initLoc2 = chrono.ChVector3d(0.0,  4.0, 0.5)     # 2nd vehicle (4 m to the right)
initRot  = chrono.ChQuaterniond(1, 0, 0, 0)

vis_type               = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE
tire_model             = veh.TireModelType_TMEASY

contact_method   = chrono.ChContactMethod_NSC
step_size        = 1e-3
render_step_size = 1.0 / 50.0            # 50 FPS

trackPoint = chrono.ChVector3d(-5.0, 0.0, 1.8)   # camera target on chassis


# -------------------------------------------------------------------------
# 3.  Create first vehicle (Sedan) – it also creates the Chrono system
# -------------------------------------------------------------------------
vehicle1 = veh.Sedan()
vehicle1.SetContactMethod(contact_method)
vehicle1.SetChassisCollisionType(chassis_collision_type)
vehicle1.SetChassisFixed(False)
vehicle1.SetInitPosition( chrono.ChCoordsysd(initLoc1, initRot) )
vehicle1.SetTireType(tire_model)
vehicle1.SetTireStepSize(step_size)
vehicle1.Initialize()

for part in [vehicle1.SetChassisVisualizationType,
             vehicle1.SetSuspensionVisualizationType,
             vehicle1.SetSteeringVisualizationType,
             vehicle1.SetWheelVisualizationType,
             vehicle1.SetTireVisualizationType]:
    part(vis_type)

system = vehicle1.GetSystem()
system.SetCollisionSystemType( chrono.ChCollisionSystem.Type_BULLET )


# -------------------------------------------------------------------------
# 4.  Create the second vehicle in the *same* physical system
# -------------------------------------------------------------------------
vehicle2 = veh.Sedan(system)                 # share the previously created ChSystem
vehicle2.SetContactMethod(contact_method)
vehicle2.SetChassisCollisionType(chassis_collision_type)
vehicle2.SetChassisFixed(False)
vehicle2.SetInitPosition( chrono.ChCoordsysd(initLoc2, initRot) )
vehicle2.SetTireType(tire_model)
vehicle2.SetTireStepSize(step_size)
vehicle2.Initialize()

for part in [vehicle2.SetChassisVisualizationType,
             vehicle2.SetSuspensionVisualizationType,
             vehicle2.SetSteeringVisualizationType,
             vehicle2.SetWheelVisualizationType,
             vehicle2.SetTireVisualizationType]:
    part(vis_type)


# -------------------------------------------------------------------------
# 5.  Terrain (concrete texture instead of tile4)
# -------------------------------------------------------------------------
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(system)
patch   = terrain.AddPatch(patch_mat,
                           chrono.ChCoordsysd( chrono.ChVector3d(0, 0, 0),
                                               chrono.ChQuaterniond(1, 0, 0, 0) ),
                           100.0, 100.0)
patch.SetTexture( veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200 )
patch.SetColor  ( chrono.ChColor(0.8, 0.8, 0.5) )
terrain.Initialize()


# -------------------------------------------------------------------------
# 6.  Irrlicht visual system – attach it to the first vehicle
# -------------------------------------------------------------------------
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Two Sedans – sinusoidal steering demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo( chrono.GetChronoDataFile('logo_pychrono_alpha.png') )
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle( vehicle1.GetVehicle() )             # camera follows car #1


# -------------------------------------------------------------------------
# 7.  Drivers
# -------------------------------------------------------------------------
# 7-A – Interactive (keyboard) driver for the first car --------------------
driver1                   = veh.ChInteractiveDriverIRR(vis)
steering_time   = 1.0
throttle_time   = 1.0
braking_time    = 0.3
driver1.SetSteeringDelta(render_step_size / steering_time)
driver1.SetThrottleDelta(render_step_size / throttle_time)
driver1.SetBrakingDelta (render_step_size / braking_time)
driver1.Initialize()


# 7-B – Automatic sinusoidal driver for the second car ---------------------
class SinusoidalDriver(veh.ChDriver):
    def __init__(self, vehicle, amp=0.4, freq=0.5, throttle=0.45):
        super(SinusoidalDriver, self).__init__(vehicle)
        self.amp      = amp
        self.freq     = freq
        self.throttle = throttle      # constant throttle

    def Synchronize(self, time):
        self.m_inputs.m_steering = self.amp * math.sin(self.freq * time)
        self.m_inputs.m_throttle = self.throttle
        self.m_inputs.m_braking  = 0.0

    def Advance(self, step):
        pass                         # nothing special to do per step

driver2 = SinusoidalDriver( vehicle2.GetVehicle() )


# -------------------------------------------------------------------------
# 8.  Simulation loop
# -------------------------------------------------------------------------
print("Vehicle 1 mass:", vehicle1.GetVehicle().GetMass())
print("Vehicle 2 mass:", vehicle2.GetVehicle().GetMass())

render_steps    = math.ceil(render_step_size / step_size)
realtime_timer  = chrono.ChRealtimeStepTimer()
step_number     = 0

while vis.Run():

    time = system.GetChTime()

    # --------------- rendering every N steps -----------------------------
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    # ---------------- Drivers -------------------------------------------
    # First car: keyboard inputs + sinusoidal steering helper
    inputs1 = driver1.GetInputs()
    inputs1.m_steering = 0.4 * math.sin(0.5 * time)    # added sinusoid

    # Second car: automatic driver updates its own inputs internally
    driver2.Synchronize(time)
    inputs2 = driver2.GetInputs()

    # ---------------- Synchronize all modules ---------------------------
    driver1.Synchronize(time)                          # (keyboard driver)
    terrain.Synchronize(time)
    vehicle1.Synchronize(time, inputs1, terrain)
    vehicle2.Synchronize(time, inputs2, terrain)
    vis.Synchronize(time, inputs1)                     # vis cares about car #1

    # ---------------- Advance simulation --------------------------------
    driver1.Advance(step_size)
    driver2.Advance(step_size)

    terrain.Advance(step_size)
    vehicle1.Advance(step_size)
    vehicle2.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)