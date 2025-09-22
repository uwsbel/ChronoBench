import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')

# If true, use provided callback to change beam visualization
vis_change = False

class MyDriver (veh.ChDriver):
	def __init__(self, vehicle, delay):
		veh.ChDriver.__init__(self, vehicle)
		self.delay = delay
	def Synchronize(self, time):
		veh.ChDriver.Synchronize(self, time)
		if (time < 0.2):
			self.SetThrottle(0)
		else:
			self.SetThrottle(0.4)
		self.SetGear(1)
		self.SetSteering(0)
def main():
    #print("Copyright (c) 2017 projectchrono.org\nChrono version: ", CHRONO_VERSION , "\n\n")

    #  Create the HMMWV vehicle, set parameters, and initialize
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE);
    hmmwv.SetChassisFixed(False); 
    hmmwv.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(-10, -2, 0.5), chrono.ChQuaterniond(1, 0, 0, 0)))
    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS);
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS);
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetTireType(veh.TireModelType_TMEASY)
    hmmwv.SetTireStepSize(1e-3)
    hmmwv.Initialize()

    hmmwv.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_NONE)
    hmmwv.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)

    hmmwv.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # Create the ground
    ground = chrono.ChBody()
    ground.SetFixed(True)
    ground.SetPos(chrono.ChVector3d(0,0,-0.3))

    # set texture
    texture = chrono.ChTexture()
    texture.SetTexture(chrono.GetChronoDataFile('textures/checker2.png'))
    #material = chrono.ChVisualMaterial()
    #texture.SetStandardTexture(material, chronox.ChVisualMaterial::TEXTURE_NONE)
    ground.AddVisualShape(chrono.ChVisualShapeTriangleMesh(texture, chrono.ChVector3d(0.01, 0, 0), chrono.ChVector3d(0, 0.01, 0), chrono.ChFramed(chrono.ChVector3d(14, -9, 0), chrono.QUNIT)))
    ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))

    # set collision
    patch_mat = chrono.ChContactMaterialNS()
    ground.AddCollisionShape(chrono.ChCollisionShapeTriangleMesh(patch_mat, # tissue material
                                                             chrono.ChVector3d(0.01, 0, 0), chrono.ChVector3d(0, 0.01, 0), chrono.ChFramed(chrono.ChVector3d(14, -9, 0), chrono.QUNIT)
                                                            ,chrono.ChVector3d(0, 16, 0), chrono.ChVector3d(24, 0, 0), chrono.ChFramed(chrono.ChVector3d(0, 0, 1.75), chrono.QUNIT)))

    # Create the Irrlicht visualization
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('HMMWV-ANCF Demo')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(hmmwv.GetVehicle())

    # Create the interactive driver
    driver = veh.ChInteractiveDriverIRR(vis)
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    driver.SetVehicle(hmmwv.GetVehicle())
    driver.Initialize()

    # Simulation loop
    while vis.Run() :
        time = hmmwv.GetSystem().GetChTime()

        # Draw beam with Irrlicht
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        # Advance simulation to generate output for next scene
        driver.Synchronize(time)
        hmmwv.Advance(step_size)
        vis.Advance(step_size)

return 0

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo. 
# If running from a different directory, you must change the path to the Chrono data directory with: 
#chrono.SetChronoDataPath('path/to/data')

main()