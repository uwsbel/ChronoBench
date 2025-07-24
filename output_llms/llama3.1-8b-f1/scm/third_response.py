import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math
import random

# =============================================================================

def main():
    #print("Copyright (c) 2017 projectchrono.org\nChrono version: ", CHRONO_VERSION , "\n\n")

    # Create systems

    #  Create the HMMWV vehicle, set parameters, and initialize
    vehicle = veh.HMMWV_Full() # veh.HMMWV_Reduced()  could be another choice here
    vehicle.SetContactMethod(contact_method)
    vehicle.SetChassisCollisionType(chassis_collision_type)
    vehicle.SetChassisFixed(False)
    vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
    vehicle.SetTireType(tire_model)
    vehicle.SetTireStepSize(tire_step_size)


    vehicle.Initialize()

    vehicle.SetChassisVisualizationType(vis_type)
    vehicle.SetSuspensionVisualizationType(vis_type)
    vehicle.SetSteeringVisualizationType(vis_type)
    vehicle.SetWheelVisualizationType(vis_type)
    vehicle.SetTireVisualizationType(vis_type)

    vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # Create the SCM deformable terrain patch
    terrain = veh.SCMTerrain(vehicle.GetSystem())
    terrain.SetSoilParameters(2e6,   # Bekker Kphi
                            0,     # Bekker Kc
                            1.1,   # Bekker n exponent
                            0,     # Mohr cohesive limit (Pa)
                            30,    # Mohr friction limit (degrees)
                            0.01,  # Janosi shear coefficient (m)
                            2e8,   # Elastic stiffness (Pa/m), before plastic yield
                            3e4    # Damping (Pa s/m), proportional to negative vertical speed (optional)
)

    # Optionally, enable moving patch feature (single patch around vehicle chassis)
    #terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(5, 3, 1))

    # Set plot type for SCM (false color plotting)
    #terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)

    # Initialize the SCM terrain (length, width, mesh resolution), specifying the initial mesh grid
    #terrain.Initialize(20, 20, 0.02)

    # Create the vehicle Irrlicht interface
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('HMMWV Demo')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(trackPoint, 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(vehicle.GetVehicle())


    # Create the driver system
    driver = veh.ChInteractiveDriverIRR(vis)

    # Set the time response for steering and throttle keyboard inputs.
    steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
    throttle_time = 1.0  # time to go from 0 to +1
    braking_time = 0.3   # time to go from 0 to +1
    driver.SetSteeringDelta(render_step_size / steering_time)
    driver.SetThrottleDelta(render_step_size / throttle_time)
    driver.SetBrakingDelta(render_step_size / braking_time)

    driver.Initialize()

    # ---------------
    # Simulation loop
    # ---------------

    # output vehicle mass
    print( "VEHICLE MASS: ",  vehicle.GetVehicle().GetMass())

    # Number of simulation steps between miscellaneous events
    render_steps = math.ceil(render_step_size / step_size)

    # Initialize simulation frame counter s
    realtime_timer = chrono.ChRealtimeStepTimer()
    step_number = 0
    render_frame = 0

    # Create a sensor manager
    sensor_manager = veh.ChSensorManager(vehicle.GetSystem())
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(12, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(22, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(32, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(42, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(52, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(62, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(72, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(82, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(92, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(102, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(112, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(122, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(132, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(142, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(152, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(162, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(172, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(182, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(192, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(202, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(212, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(222, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(232, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(242, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(252, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(262, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(272, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(282, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(292, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(302, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(312, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(322, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(332, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(342, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(352, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(362, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(372, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(382, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(392, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(402, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(412, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(422, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(432, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(442, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(452, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(462, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(472, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(482, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(492, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(502, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(512, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(522, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(532, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(542, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(552, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(562, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(572, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(582, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(592, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(602, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(612, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(622, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(632, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(642, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(652, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(662, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(672, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(682, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(692, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(702, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(712, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(722, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(732, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(742, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(752, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(762, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(772, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(782, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(792, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(802, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(812, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(822, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(832, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(842, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(852, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(862, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(872, 2.5, 100, 1000, 1000, 1000))
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(882, 2.5, 100, 1000, 1000, 1000))
    sensor