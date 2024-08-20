import pychrono as pc
import pychrono.beams as pb
import pychrono.visualization as pviz


pc.Initialize()


system = pc.PhysicalSystem()


beam_length = 1.0
beam_width = 0.1
beam_height = 0.05
beam_material = pb.Material(YoungModulus=1e7, PoissonRatio=0.3, Density=1000)


beam_mesh = pb.BeamMesh.CreateUniform(beam_length, beam_width, beam_height, 10)
beam_body = pb.BeamBody(beam_mesh, beam_material)
system.Add(beam_body)


beam_body.SetPos(pc.Vec3(0, 0, 0))
beam_body.SetRot(pc.Quat.FromEulerAngles(pc.Vec3(0, 0, pc.DegToRad(90))))


node_mass = 1.0
node_inertia = pc.Matrix33.Identity() * node_mass / 3.0
node_body = pc.Body(node_mass, node_inertia)
system.Add(node_body)


node_body.SetPos(pc.Vec3(beam_length / 2, 0, 0))


joint = pc.JointRevolute(node_body, beam_body, pc.Vec3(beam_length / 2, 0, 0), pc.Vec3(0, 0, 1))
system.Add(joint)


viz = pviz.IrrlichtVisualizer(system)
viz.CreateWindow(800, 600, "PyChrono Beam Simulation")
viz.SetCameraPos(pc.Vec3(2, 2, 2))


simulation_time = 5.0
time_step = 0.01
for t in pc.range(simulation_time, time_step):
    
    node_body.ApplyForce(pc.Vec3(0, 0, -1.0), pc.Vec3(0, 0, 0))

    
    system.Step(time_step)

    
    viz.Update()


pc.Finalize()