import pychrono as ch
    import pychrono.irrlicht as chir
    import pychrono.fea as chfe
    import pychrono.pardisomkl as mkl

    
    sys = ch.ChSystemNSC()

    
    mesh = chfe.ChMesh()

    
    node1 = chfe.ChNodeFE(mesh)
    node1.SetCoords(ch.ChVectorD(0, 0, 0))
    node1.SetPosDt(ch.ChVectorD(0, 0, 0))
    node1.SetRotDt(ch.QuatFromAngleAxis(0, ch.ChVectorD(0, 0, 1)))

    
    node2 = chfe.ChNodeFE(mesh)
    node2.SetCoords(ch.ChVectorD(0.1, 0, 0))
    node2.SetPosDt(ch.ChVectorD(0.1, 0, 0))
    node2.SetRotDt(ch.QuatFromAngleAxis(0, ch.ChVectorD(0, 0, 1)))

    
    node3 = chfe.ChNodeFE(mesh)
    node3.SetCoords(ch.ChVectorD(0.2, 0, 0))
    node3.SetPosDt(ch.ChVectorD(0.2, 0, 0))
    node3.SetRotDt(ch.QuatFromAngleAxis(0, ch.ChVectorD(0, 0, 1)))

    
    node4 = chfe.ChNodeFE(mesh)
    node4.SetCoords(ch.ChVectorD(0.3, 0, 0))
    node4.SetPosDt(ch.ChVectorDt(0.3, 0, 0))
    node4.SetRotDt(ch.QuatFromAngleAxis(0, ch.ChVectorD(0, 0, 1)))

    
    node5 = chfe.ChNodeFE(mesh)
    node5.SetCoords(ch.ChVectorD(0.4, 0, 0))
    node5.SetPosDt(ch.ChVectorD(0.4, 0, 0))
    node5.SetRotDt(ch.QuatFromAngleAxis(0, ch.ChVectorD(0, 0, 1)))

    
    node6 = chfe.ChNodeFE(mesh)
    node6.SetCoords(ch.ChVectorD(0.5, 0, 0))
    node6.SetPosDt(ch.ChVectorD(0.5, 0, 0))
    node6.SetRotDt(ch.QuatFromAngleAxis(0, ch.ChVectorD(0, 0, 1)))

    
    node7 = chfe.ChNodeFE(mesh)
    node7.SetCoords(ch.ChVectorD(0.6, 0, 0))
    node7.SetPosDt(ch.ChVectorD(0.6, 0, 0))
    node7.SetRotDt(ch.QuatFromAngleAxis(0, ch.ChVectorD(0, 0, 1)))

    
    node8 = chfe.ChNodeFE(mesh)
    node8.SetCoords(ch.ChVectorD(0.7, 0, 0))
    node8.SetPosDt(ch.ChVectorD(0.7, 0, 0))
    node8.SetRotDt(ch.QuatFromAngleAxis(0, ch.ChVectorD(0, 0, 1)))

    
    node9 = chfe.ChNodeFE(mesh)
    node9.SetCoords(ch.ChVectorD(0.8, 0, 0))
    node9.SetPosDt(ch.ChVectorD(0.8, 0, 0))
    node9.SetRotDt(ch.QuatFromAngleAxis(0, ch.ChVectorD(0, 0, 1)))

    
print("error happened with only start ```python")