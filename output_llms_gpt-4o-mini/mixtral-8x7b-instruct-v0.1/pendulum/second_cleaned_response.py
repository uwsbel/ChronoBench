if log_info and sys.GetChTime() > 1:
    pos_1 = pend_1.GetPos()  
    print("t = ", sys.GetChTime())
    print("     ", pos_1.x, "  ", pos_1.y)
    lin_vel_1 = pend_1.GetPosDt()  
    print("     ", lin_vel_1.x, "  ", lin_vel_1.y)
    log_info = False