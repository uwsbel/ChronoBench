import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')

# If true, use provided callback to change camera's horizontal and vertical angles
viskin_camera = True

class MyReportContact(chrono.ReportContactCallback):
    def __init__(self):
        chrono.ReportContactCallback.__init__(self)
    def OnReportContact(self,vA,vB,cA,dist,rad,force,torque,modA,modB):
        bodA = chrono.CastContactable_to_ChBody(modA)
        bodB = chrono.CastContactable_to_ChBody(modB)
        print('  contact (A: {}   B: {})'.format(bodA.GetName(), bodB.GetName()))
        return True

def main():
    #print("Copyright (c) 2017 projectchrono.org\nChrono version: ", CHRONO_VERSION , "\n\n")

    #  Create the simulation system and add items
    sys = chrono.ChSystemNSC()
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # Create ground body
    ground = chrono.ChBodyEasyBox(1000,20,3, 1000)
    ground.SetPos(chrono.ChVector3d(0,0.0,-0.2))
    ground.SetFixed(True)
    ground.GetVisualShape(0).SetTexture(veh.GetDataFile('textures/tile4.jpg'))
    sys.Add(ground);

    # Create pendulum body
    pendulum = chrono.ChBodyEasyBox(1,1,1, 1000)
    pendulum.SetPos(chrono.ChVector3d(0,5,0))
    pendulum.SetRot(chrono.QuatFromAngleAxis(.2,chrono.ChVector3d(0,0,1)))
    pendulum.SetMass(20)
    pendulum.GetVisualShape(0).SetTexture(veh.GetDataFile('textures/ramp_diffuse.png'))
    sys.Add(pendulum);

    # Create revolute joint between pendulum and ground
    link = chrono.ChLinkLockRevolute()
    link.Initialize(pendulum,         # body A in link
                    ground,           # body C in link
                    chrono.ChFramed(chrono.ChVector3d(0,1,-0.1),chrono.QuatFromAngleAxis(.2,chrono.ChVector3d(0,0,1))))
    sys.Add(link);

    # Create a large cube, as an obstacle
    obstacle = chrono.ChBodyEasyBox(3,3,1, 1000)
    obstacle.SetPos(chrono.ChVector3d(0,-1,-0.6))
    obstacle.SetFixed(True)
    obstacle.GetVisualShape(0).SetTexture(veh.GetDataFile('textures/ramp_diffuse.png'))
    sys.AddBody(obstacle);

    # Create a pyramid made by a few bodies connected by spherical joints
    # and add it to a ChBodyContainerAssumed, that will manage all sub-bodies automatically.
    ass = chrono.ChBodyContainerAssumed()
    sys.Add(a);

    # Create the Irrlicht visualization
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024,768)
    vis.SetWindowTitle('Test multiple collision systems')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.5,0.2,-0.5))
    vis.AddTypicalLights()

    vis.AddChAll();

    # Set the output directory (will be created if does not exist)
    try:
        vis.SetOutputPath(chrono.GetChronoDataFile(''))
    except:
        print("Warning. Output directory does not exist. Output will be saved in the current directory.")
        vis.SetOutputPath('./')

    # Generate output files
    if (vis.Run()):
        print('Output generated as png images');

    # Run the simulation
    t1 = 0
    ch_time = sys.GetChTime()

    while (vis.Run()) :
        sys.DoStepDynamics(1e-4);

        time = sys.GetChTime()

        if (time - ch_time >= 1.0) :
            print('Pendulum position = ', end='')
            print(pendulum.GetPos().x, end=' ')
            print(pendulum.GetPos().y, end=' ')
            print(pendulum.GetPos().z)
            print('Pendulum velocity =', pendulum.GetPosDt().x, pendulum.GetPosDt().y, pendulum.GetPosDt().z)
            ch_time = time

        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        help1 = '  ' \
                '\nCamera Zoom:    Mouse wheel or +/- keys' \
                '\nCamera Rotate:   Left/right arrow keys or mouse movement' \
                '\nCamera Pan:      Up/down or shift + left/right arrow keys'

    return 0

main()