import pychrono as chrono
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath('path_to_your_chrono_data_folder')
my_system = chrono.ChSystemNSC()


my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


beam_length = 1.0
num_elements = 10
element_length = beam_length / num_elements

for i in range(num_elements):
    element = chrono.ChElementCableANCF()
    element.SetLength(element_length)
    element.SetDiameter(0.01)
    element.SetYoungModulus(2e5)
    element.SetBeamRaleyghDamping(0.001)
    element.SetBeamSectionAsCircular(0.005)
    my_system.Add(element)

    if i == 0:
        
        node_ground = chrono.ChNodeBodyEasySphere(0.02, 1000, True, True)
        my_system.Add(node_ground)
        my_system.Add(chrono.ChLinkLockLock(node_ground, element.GetNodeA()))
    else:
        
        my_system.Add(chrono.ChLinkPointPoint(element.GetNodeA(), previous_element.GetNodeB()))

    previous_element = element


my_application = chronoirr.ChIrrApp(my_system, 'Beam ANCF', chronoirr.dimension2du(1024, 768))


my_application.SetCameraVertical(chrono.CameraVerticalDir.Z)
my_application.SetCameraPosition(chrono.ChVectorD(0, -2, 0))
my_application.SetCameraTarget(chrono.ChVectorD(0, 0, 0))


while my_application.GetDevice().run():
    my_application.BeginScene()
    my_application.DrawAll()
    my_application.DoStep()
    my_application.EndScene()