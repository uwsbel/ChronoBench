import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr
import math

# =============================================================================

def main():
    #print("Copyright (c) 2017 projectchrono.org\nChrono version: ", CHRONO_VERSION , "\n\n")

    # --------------------------
    # Create the ChSystem
    # --------------------------
    sys = chrono.ChSystemNSC()
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0,0,-9.81))

    # -------------------------------------
    # Add a mesh to be sensed by the lidar
    # -------------------------------------
    # Load a mesh from a Wavefront .obj file
    mesh = chrono.ChTriangleMeshConnected()
    mesh.LoadWavefrontMesh(chrono.GetChronoDataFile('models/lidar_target.obj'), 1, 0.1)
    # Create a body from the mesh
    bodyA = chrono.ChBody()
    bodyA.SetPos(chrono.ChVector3d(0, 0, 0))
    bodyA.SetFixed(True)
    bodyA.GetVisualShape(0).SetMesh(mesh)
    bodyA.EnableCollision(0)
    sys.Add(bodyA)

    # ------------------------------------------------
    # Create a sensor manager and add a lidar sensor
    # ------------------------------------------------
    # Create a sensor manager
    sens_manager = sens.ChSensorManager(sys)
    # Create a point light for illumination
    sens_manager.AddPointLight(chrono.ChVector3f(2,2.2,2000), chrono.ChColor(2,2.2,2))
    # Create a lidar sensor
    offset_pose = chrono.ChFramed(chrono.ChVector3d(-8, 0, 2), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
    lidar = sens.ChLidarSensor(bodyA, offset_pose, chrono.LidarSensorDataTypes_MESH,  # data type
                               360,  # horizontal angle
                               0, 0,  # vertical start and end angles
                               10,  # horizontal resolution
                               0.01, 0.01,  # horizontal and vertical resolution
                               5,  # max lidar range
                               0.005)  # noise
    # Add noise filters
    lidar.PushFilter(sens.ChFilterLidarNoise(lidar, 0.01))
    lidar.PushFilter(sens.ChFilterVisualize(1000))
    lidar.PushFilter(sens.ChFilterSave(1000, chrono.GetChronoDataFile("lidar_data/"), "out"))
    # Add the sensor to the sensor manager
    sens_manager.AddSensor(lidar)
    # Create and initialize an Irrlicht application
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('Lidar Demo')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.5,2.5,0), chrono.ChVector3d(0,0,0))
    vis.AddTypicalLights()
    # Simulation loop
    time = 0
    time_step = 5e-3
    time_end = 15
    lidar_offset = chrono.ChFramed(chrono.ChVector3d(-8, 0, 2), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
    orbit_radius = 10
    orbit_rate = 0.2
    orbit_time = 0
    while time < time_end:
        time = sys.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        # Update offset pose for the lidar
        orbit_angle = orbit_time * orbit_rate
        lidar_offset.SetPos(chrono.ChVector3d(-8 * math.cos(orbit_angle), -8 * math.sin(orbit_angle), 2))
        lidar.SetOffsetPose(lidar_offset)

        # Update sensor manager
        sens_manager.Update()
        sys.DoStepDynamics(time_step)

        orbit_time += time_step

    return 0

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')

main()