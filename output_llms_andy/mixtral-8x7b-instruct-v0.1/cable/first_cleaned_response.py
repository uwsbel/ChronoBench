import chrono
from chrono.fea import crfea
from chrono.irrlicht import cr_irr_app


chrono.SetChronoDataPath('path/to/chrono/data')
my_system = chrono.ChSystemSMC()


beam_len = 10.0  
beam_width = 0.5  
beam_height = 0.5  
num_elements = 10  

beam_node_positions = []
for i in range(num_elements + 1):
    x = i * beam_len / num_elements
    beam_node_positions.append([x, 0, 0])

beam_body = crfea.ChBodyEasyCableANCF(
    my_system,
    vertex_positions=beam_node_positions,
    vertex_masses=[1.0] * (num_elements + 1),
    cable_width=beam_width,
    cable_height=beam_height,
    youngs_modulus=200e6,
    poisson_ratio=0.3,
    density=7800,
)
beam_body.SetPos(chrono.ChVectorD(0, 1, 0))
my_system.AddBody(beam_body)


hinge_joint = chrono.ChLinkMateSocket.ChLinkMateSocket()
hinge_joint.Initialize(beam_body, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
hinge_joint.SetFixed(True)
my_system.AddLink(hinge_joint)


my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


my_vis = cr_irr_app.ChIrrApp(my_system, 'ANCF Beam Simulation', chrono.ChVectorD(0, 0, 0))
my_vis.AddLogo()
my_vis.AddSkyBox()
my_vis.AddCamera(chrono.ChVectorD(beam_len / 2, beam_len, 5 * beam_len))
my_vis.AddTypicalLights()


beam_shape = my_vis.AddCable(beam_body, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(beam_len, 0, 0))
beam_shape.SetColor(chrono.ChColor(0.5, 0.5, 1.0))


for i in range(num_elements + 1):
    node_pos = beam_body.GetCableShape().GetCable().GetVertex(i).GetPos()
    my_vis.AddSphere(node_pos, 0.1, 20, chrono.ChColor(1.0, 0.0, 0.0))


my_vis.SetTimestep(1e-4)
while my_vis.Run():
    my_system.DoStepDynamics(my_vis.GetTimerStep())
    my_vis.Wait()


my_system.SaveCSV('beam_final_state.csv')