import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# -----------------------------------------------------------------------------
# Utility struct for driver inputs (mimics chrono::vehicle::ChDriver::DriverInputs)
# -----------------------------------------------------------------------------
class DriverInputs:
    def __init__(self):
        self.m_throttle = 0.0
        self.m_steering = 0.0
        self.m_braking = 0.0
        # gear up/down not used here
        self.m_gear_up = False
        self.m_gear_down = False

# -----------------------------------------------------------------------------
# Pure‐pursuit path‐following driver with PID steering + constant throttle
# -----------------------------------------------------------------------------
class PathDriver:
    def __init__(self, vehicle, path_points, look_ahead_dist, step_size):
        self.vehicle = vehicle
        self.path = path_points
        self.L = look_ahead_dist
        self.dt = step_size
        # PID gains (tune as needed)
        self.Kp = 0.8
        self.Ki = 0.0
        self.Kd = 0.1
        self.err_int = 0.0
        self.err_prev = 0.0
        # persistent holder for inputs
        self.inputs = DriverInputs()
        # last computed points (for visualization)
        self.current_sentinel = chrono.ChVector3d(0,0,0)
        self.current_target   = chrono.ChVector3d(0,0,0)

    def GetInputs(self):
        # return the same object each time
        return self.inputs

    def Synchronize(self, time):
        # 1) get vehicle position & heading (yaw)
        pos = self.vehicle.GetVehicle().GetPos()
        self.current_sentinel = pos
        q   = self.vehicle.GetVehicle().GetRot()
        # quaternion → Euler123, returns a ChVector with (roll,pitch,yaw)
        euler = q.Q_to_Euler123()
        yaw   = euler.z

        # 2) find the first path point farther than look-ahead distance
        target = None
        for P in self.path:
            d = math.hypot(P.x - pos.x, P.y - pos.y)
            if d > self.L:
                target = P
                break
        if target is None:
            # wrap around
            target = self.path[0]
        self.current_target = target

        # 3) compute heading error
        desired_yaw = math.atan2(target.y - pos.y, target.x - pos.x)
        err = desired_yaw - yaw
        # wrap to [-pi,pi]
        while err >  math.pi: err -= 2*math.pi
        while err < -math.pi: err += 2*math.pi

        # 4) PID
        self.err_int  += err * self.dt
        derr = (err - self.err_prev) / self.dt
        delta = self.Kp*err + self.Ki*self.err_int + self.Kd*derr
        self.err_prev = err

        # 5) clamp steering to [-1,1]
        delta = max(-1.0, min(1.0, delta))

        # 6) fill inputs
        self.inputs.m_steering = delta
        self.inputs.m_throttle = 0.3
        self.inputs.m_braking  = 0.0

    def Advance(self, step):
        # nothing to do
        pass

# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------
if __name__ == "__main__":
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

    # contact method
    contact_method = chrono.ChContactMethod_NSC

    # -----------------------------------------------------------------------------
    # Create the HMMWV vehicle
    # -----------------------------------------------------------------------------
    initLoc = chrono.ChVector3d(20, 0, 0.5)    # start at angle=0 on circle radius=20
    initRot = chrono.ChQuaterniond(1,0,0,0)

    vehicle = veh.HMMWV_Full()
    vehicle.SetContactMethod(contact_method)
    vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
    vehicle.SetChassisFixed(False)
    vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
    vehicle.SetTireType(veh.TireModelType_TMEASY)
    vehicle.SetTireStepSize(1e-3)
    vehicle.Initialize()
    # visualization
    vis_type = veh.VisualizationType_PRIMITIVES
    vehicle.SetChassisVisualizationType(vis_type)
    vehicle.SetSuspensionVisualizationType(vis_type)
    vehicle.SetSteeringVisualizationType(vis_type)
    vehicle.SetWheelVisualizationType(vis_type)
    vehicle.SetTireVisualizationType(vis_type)
    vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # -----------------------------------------------------------------------------
    # Create terrain (200 × 100)
    # -----------------------------------------------------------------------------
    terrain = veh.RigidTerrain(vehicle.GetSystem())
    mat = chrono.ChContactMaterialNSC()
    mat.SetFriction(0.9)
    mat.SetRestitution(0.01)
    patch = terrain.AddPatch(
        mat,
        chrono.ChCoordsysd(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(1,0,0,0)),
        length=200.0, width=100.0)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8,0.8,0.5))
    terrain.Initialize()

    # -----------------------------------------------------------------------------
    # Irrlicht visualization
    # -----------------------------------------------------------------------------
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('HMMWV Path Following')
    vis.SetWindowSize(1280,1024)
    vis.SetChaseCamera(chrono.ChVector3d(-3,0,1.1), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()

    vis.AttachVehicle(vehicle.GetVehicle())

    # -----------------------------------------------------------------------------
    # Build and visualize a circular path
    # -----------------------------------------------------------------------------
    radius = 20.0
    Npts = 100
    path_pts = []
    for i in range(Npts):
        th = 2*math.pi * i / Npts
        # z = 0.5 so the sphere is just above ground
        p = chrono.ChVector3d(radius*math.cos(th), radius*math.sin(th), 0.5)
        path_pts.append(p)
        # draw small red spheres for the whole path
        node = vis.GetSceneManager().addSphereSceneNode(0.3, 16)
        node.setPosition(irr.vector3df(p.x, p.y, p.z))
        node.setMaterialFlag(irr.EMF_LIGHTING, False)
        node.setMaterialTexture(0, None)
        node.setMaterialFlag(irr.EMF_ZBUFFER, True)
        node.setMaterialFlag(irr.EMF_BACK_FACE_CULLING, False)
        node.getMaterial().setDiffuseColor(irr.SColor(255,200,50,50))

    # Two dynamic spheres: sentinel (blue) & target (green)
    sentinel_node = vis.GetSceneManager().addSphereSceneNode(0.5, 16)
    sentinel_node.setMaterialFlag(irr.EMF_LIGHTING, False)
    sentinel_node.getMaterial().setDiffuseColor(irr.SColor(255, 50,50,200))

    target_node = vis.GetSceneManager().addSphereSceneNode(0.5, 16)
    target_node.setMaterialFlag(irr.EMF_LIGHTING, False)
    target_node.getMaterial().setDiffuseColor(irr.SColor(255, 50,200,50))

    # -----------------------------------------------------------------------------
    # Create path-following driver
    # -----------------------------------------------------------------------------
    lookahead = 5.0
    step_size = 1e-3
    driver = PathDriver(vehicle, path_pts, lookahead, step_size)

    print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

    # timing
    render_step_size = 1.0/50
    render_steps = math.ceil(render_step_size / step_size)
    realtime_timer = chrono.ChRealtimeStepTimer()

    step_number = 0

    # -----------------------------------------------------------------------------
    # Run the simulation
    # -----------------------------------------------------------------------------
    while vis.Run():
        t = vehicle.GetSystem().GetChTime()

        # render
        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        # controller update
        driver.Synchronize(t)
        terrain.Synchronize(t)
        vehicle.Synchronize(t, driver.GetInputs(), terrain)
        vis.Synchronize(t, driver.GetInputs())

        # update sentinel & target spheres
        ps = driver.current_sentinel
        pt = driver.current_target
        sentinel_node.setPosition(irr.vector3df(ps.x, ps.y, ps.z+0.2))
        target_node.setPosition( irr.vector3df(pt.x, pt.y, pt.z+0.2))

        # advance all modules
        driver.Advance(step_size)
        terrain.Advance(step_size)
        vehicle.Advance(step_size)
        vis.Advance(step_size)

        step_number += 1
        realtime_timer.Spin(step_size)