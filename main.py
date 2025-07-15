from module.functions import *
# ===========================
# CHƯƠNG TRÌNH CHÍNH
# ===========================
var_ref_flag  = False
index         = []
id_flag       = False
t_for_xy_1    = 5
t_for_xy_2    = 10
pid_1 = {'PR': 0.15, 'IR': 0.15, 'DR': 0.01, 'PP' : 0.15, 'IP' : 0.15, 'DP' : 0.01}
pid_2 = {'PR': 0.135, 'IR': 0.135, 'DR': 0.0036, 'PP' : 0.135, 'IP' : 0.135, 'DP' : 0.0036}
model, labels = load_model()
print("load model")
if not model or not labels:
    print("can not load model")

# cap = cv2.VideoCapture(0)
# print("chuan bi camera")
# while not cap.isOpened():
#     print("waiting for open cam")

port = 5001
print("mo port:", port)
vehicle = connect_drone()
threading.Thread(target=send_position_to_host, args=("192.168.32.113", 9999), daemon=True).start()
print("da gui, xuong duoi")
# set_pid_profile(pid_2)
# time.sleep(3)
while True:
    TT               = 10
    Start            = False
    flag_xy          = False
    flag_alt         = False
    flag_pid         = False
    dungim           = False
    Target_locked    = False
    Run_servo        = False
    khong_thay_toa_do_muc_tieu = False
    global x, y, vx, vy, dist
    # frame_center_x, frame_center_y = None, None
    docao      = 3
    vitrihomex = 0
    vitrihomey = 0
    vitritram1_x = 4
    vitritram1_y = 0
    vitritram2_x = 4
    vitritram2_y = 2
    Start, thu_tu_tram = lis_host(vitritram1_x, vitritram1_y, vitritram2_x, vitritram2_y, port)
    print(thu_tu_tram)

    if thu_tu_tram[0] == 1:
        tram_x = abs(vitritram1_x/t_for_xy_1)     # vel of s1
        tram_y = abs(vitritram1_y/t_for_xy_1)     # vel of s1
        vel_x  = vitritram1_x/t_for_xy_1
        vel_y  = vitritram1_y/t_for_xy_1
        vitritram_x = vitritram1_x
        vitritram_y = vitritram1_y
        t_percent   = t_for_xy_1 * 0.9
    elif thu_tu_tram[0] == 2:
        tram_x = abs(vitritram2_x/t_for_xy_2)     # vel for s2
        tram_y = abs(vitritram2_y/t_for_xy_2)     # vel for s2
        vel_x  = vitritram2_x/t_for_xy_2
        vel_y  = vitritram2_y/t_for_xy_2
        vitritram_x = vitritram2_x
        vitritram_y = vitritram2_y
        t_percent   = t_for_xy_2 * 0.9

    dist_set = math.sqrt(vitritram_x**2 + vitritram_y**2)
    home     = math.sqrt(vitrihomex**2 + vitrihomey**2)
    
    if not Start:
        while True:
            if Start: break
    if Start:
        cap = cv2.VideoCapture(0)
        print("chuan bi camera")
        while not cap.isOpened():
             print("waiting for open cam")
        log_filename = f"flight_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        with open(log_filename, 'w') as f:
           f.write("timestamp,x,y,z,vx,vy,vz\n")  # Ghi header
        # vehicle = connect_drone()
        arm_and_takeoff(vehicle, docao)
        if not flag_alt:
            alt_set = luu_do_cao(docao)
            flag_alt = True
#             bay_only_vel(vehicle, -10)
        # vehicle.airspeed = 5 #m/s
        # vehicle.groundspeed = 1 #m/s
        
        kp_roll=0.5
        ki_roll=0
        kd_roll=0

        kp_pitch=0.5
        ki_pitch=0
        kd_pitch=0

        kp_z    = 3.3
        ki_z    = 0.0035
        kd_z    = 0.0001
        
        kp_z2    = 1
        ki_z2    = 0
        kd_z2    = 0
        
        pidz1 = PID_onlyZ(kp_z, ki_z, kd_z, alt_set)
        while True:
            dist = distance()
            # print(dist)
            vel_z = controlforZ(pidzz=pidz1, altreal=vehicle.location.global_relative_frame.alt)
            bay_xyz(vehicle, 0, 0, 0, vel_x, vel_y, -vel_z, log_filename)
            
            if dist >= dist_set*0.9:
                print("bay tiep", dist)
                coef_ax = path_planning(0,0,tram_x,0,t_percent)   # x0, xf, vx0, vxf, t
                coef_ay = path_planning(0,0,tram_y,0,t_percent)   # y0, yf, vy0, vyf, t
                for ttime in np.arange(0,TT,0.1):
                    x, vx = pos_xyz(coef_ax, ttime)
                    y, vy = pos_xyz(coef_ay, ttime)
                    vel_z = controlforZ(pidzz=pidz1, altreal=vehicle.location.global_relative_frame.alt)
                    if vx < 5e-2:
                        break
                    bay_xyz(vehicle, 0, 0, 0, vx, vy, -vel_z, log_filename)
                    time.sleep(0.01)
                if not dungim:
                    # Sau khi bay den diem
                    bay_xyz(vehicle, 0, 0, 0, 0, 0, 0, log_filename)
#                     time.sleep(0.01)
                    dungim = True
                if dungim:
                    break
            time.sleep(0.01)

        if dungim:
            print("mo camera")
            counting = 0
            frame = None
            frame_center_x, frame_center_y = None, None
            # cap = cv2.VideoCapture(0)
            # while not cap.isOpened():
            #     print("waiting for open cam")
            cap.set(3, 640)
            cap.set(4, 640)
#             model, labels = load_model()
#             if not model or not labels:
#                  print("can not load model")
#             time.sleep(0.5)
            
            while True:
                bbox_colors = [(164,120,87), (68,148,228), (93,97,209), (178,182,133), (88,159,106), 
                (96,202,231), (159,124,168), (169,162,241), (98,118,150), (172,176,184)]
                
                ret, frame = cap.read()
                if not ret or frame is None:
                    print("No frame")

                results = imgprocess(frame,model,labels)
                frame_center_x, frame_center_y, box_center_x, box_center_y = results
                if not flag_pid:
                    pidroll, pidpitch, pidz2 = PIDcontrol(kp_roll, ki_roll, kd_roll, kp_pitch, ki_pitch, kd_pitch, kp_z2, ki_z2, kd_z2, setpoint_rll=frame_center_x, setpoint_ptc=frame_center_y, setpoint_z=alt_set)
                    flag_pid = True
                    
                if results is None:
                    print("Press q and exit")
                    khong_thay_toa_do_muc_tieu = True
                    break
                
                if None in (frame_center_x, frame_center_y, box_center_x, box_center_y):
                    print("do not know x and y")
                    counting += 1
#                     wait_time_img(10)
                    if counting == 20:
                        dungim = False
                        # rtl(vehicle)
                        break
                    continue
                else:
                    counting = 0
                    
                ctr_signal_roll, ctr_signal_pitch, ctr_signal_z = controller(box_center_x, box_center_y, vehicle.location.global_relative_frame.alt, pidroll, pidpitch, pidz2)
                # print("box_center_x:", box_center_x)
                # print("box_center_y:", box_center_y)
                # print("frame_center_x:", frame_center_x)
                # print("frame_center_y:", frame_center_y)
                if box_center_x >= frame_center_x and box_center_y > frame_center_y:
                     print("bay 44")
                     bay_xyz(vehicle, 0, 0, -ctr_signal_z, -ctr_signal_roll, ctr_signal_pitch, 0, log_filename)
                    
                if box_center_x < frame_center_x and box_center_y > frame_center_y:
                     print("bay 43")
                     bay_xyz(vehicle, 0, 0, -ctr_signal_z, -ctr_signal_roll, -ctr_signal_pitch, 0, log_filename)
                    
                if box_center_x < frame_center_x and box_center_y < frame_center_y:
                     print("bay 42")
                     bay_xyz(vehicle, 0, 0, -ctr_signal_z, ctr_signal_roll, -ctr_signal_pitch, 0, log_filename)
                    
                if box_center_x > frame_center_x and box_center_y < frame_center_y:
                     print("bay 41")
                     bay_xyz(vehicle, 0, 0, -ctr_signal_z, ctr_signal_roll, ctr_signal_pitch, 0, log_filename)
#                 bay_xyz(vehicle, 0, 0, 0, -ctr_signal_roll, -ctr_signal_pitch, -ctr_signal_z)
                if box_center_x >= (frame_center_x-(frame_center_x*0.15)) and box_center_x <= (frame_center_x+(frame_center_x*0.15)):
                    if box_center_y >= (frame_center_y-(frame_center_y*0.15)) and box_center_y <= (frame_center_y+(frame_center_y*0.15)):
                        bay_xyz(vehicle, 0, 0, 0, 0, 0, 0, log_filename)
                        print("target locked")
                        Target_locked = True
                        # time_start = time.time()
                        # while time.time() - time_start < 5:
                        time.sleep(1)
                        Run_servo = True
                        if Target_locked:
                            bay_xyz(vehicle, 0, 0, alt_set, 0, 0, 0, log_filename)
                            # ha_canh(vehicle)
                            if Run_servo:
                                servo(10, 2000)
                                servo(11, 900)
                                servo(12, 800)
                                # wait_time_servo(30, alt_set)
                                time.sleep(20)
                                bay_xyz(vehicle, 0, 0, -alt_set, 0, 0, 0, log_filename)
                                break
                                # Stop_servo = 1
#                         Stop_servo = lis_esp()
                        # if Stop_servo:
                        #     servo(11,1500)
                        #     wait_time_servo(10)
                        #     servo(11,2200)
                        #     wait_time_servo(20)
                        #     break
                time.sleep(0.1)
            # set_pid_profile(pid_1)
            # time.sleep(3)
            cap.release()
            cv2.destroyAllWindows()
                
        # ha_canh(vehicle)

        rtl(vehicle)
        #####################################################################################################
#         flag_pid = False
#         dungim   = False
#         while True:
#             if khong_thay_toa_do_muc_tieu or counting>=20:
#                 print("out")
#                 break

#             dist = distance()
#             # print(dist)
#             vel_z = controlforZ(pidzz=pidz1, altreal=vehicle.location.global_relative_frame.alt)
#             bay_xyz(vehicle, 0, 0, 0, -vel_x, -vel_y, -vel_z, log_filename)
            
#             if dist <= home+0.5:
#                 print("bay tiep", dist)
#                 coef_ax = path_planning(0,0,tram_x,0,t_percent)   # x0, xf, vx0, vxf, t
#                 coef_ay = path_planning(0,0,tram_y,0,t_percent)   # y0, yf, vy0, vyf, t
#                 for ttime in np.arange(0,TT,0.1):
#                     x, vx = pos_xyz(coef_ax, ttime)
#                     y, vy = pos_xyz(coef_ay, ttime)
#                     vel_z = controlforZ(pidzz=pidz1, altreal=vehicle.location.global_relative_frame.alt)
#                     if vx < 5e-2:
#                         break
#                     bay_xyz(vehicle, 0, 0, 0, -vx, -vy, -vel_z, log_filename)
#                     time.sleep(0.01)
#                 if not dungim:
#                     # Sau khi bay den diem
#                     bay_xyz(vehicle, 0, 0, 0, 0, 0, 0, log_filename)
#                     time.sleep(0.1)
#                     dungim = True
#                 if dungim:
#                     break
#             time.sleep(0.01)

#         if dungim:
#             print("mo camera")
#             counting = 0
#             frame = None
#             frame_center_x, frame_center_y = None, None
#             cap = cv2.VideoCapture(0)
#             while not cap.isOpened():
#                 print("Can not open camera")
#             cap.set(3, 640)
#             cap.set(4, 640)

#             while True:
#                 bbox_colors = [(164,120,87), (68,148,228), (93,97,209), (178,182,133), (88,159,106), 
#                 (96,202,231), (159,124,168), (169,162,241), (98,118,150), (172,176,184)]
                
#                 ret, frame = cap.read()
#                 if not ret or frame is None:
#                     print("No frame")

#                 results = imgprocess(frame,model,labels)
#                 frame_center_x, frame_center_y, box_center_x, box_center_y = results
#                 if not flag_pid:
#                     pidroll, pidpitch, pidz2 = PIDcontrol(kp_roll, ki_roll, kd_roll, kp_pitch, ki_pitch, kd_pitch, kp_z, ki_z, kd_z, setpoint_rll=frame_center_x, setpoint_ptc=frame_center_y, setpoint_z=alt_set)
#                     flag_pid = True
                    
#                 if results is None:
#                     print("Press q and exit")
#                     khong_thay_toa_do_muc_tieu = True
#                     break
                
#                 if None in (frame_center_x, frame_center_y, box_center_x, box_center_y):
#                     print("do not know x and y")
#                     counting += 1
# #                     wait_time_img(10)
#                     if counting == 20:
#                         dungim = False
#                         # rtl(vehicle)
#                         break
#                     continue
#                 else:
#                     counting = 0
                    
#                 ctr_signal_roll, ctr_signal_pitch, ctr_signal_z = controller(box_center_x, box_center_y, vehicle.location.global_relative_frame.alt, pidroll, pidpitch, pidz2)
#                 # print("box_center_x:", box_center_x)
#                 # print("box_center_y:", box_center_y)
#                 # print("frame_center_x:", frame_center_x)
#                 # print("frame_center_y:", frame_center_y)
#                 if box_center_x >= frame_center_x and box_center_y > frame_center_y:
#                      print("bay 44")
#                      bay_xyz(vehicle, 0, 0, -ctr_signal_z, -ctr_signal_roll, ctr_signal_pitch, 0, log_filename)
                    
#                 if box_center_x < frame_center_x and box_center_y > frame_center_y:
#                      print("bay 43")
#                      bay_xyz(vehicle, 0, 0, -ctr_signal_z, -ctr_signal_roll, -ctr_signal_pitch, 0, log_filename)
                    
#                 if box_center_x < frame_center_x and box_center_y < frame_center_y:
#                      print("bay 42")
#                      bay_xyz(vehicle, 0, 0, -ctr_signal_z, ctr_signal_roll, -ctr_signal_pitch, 0, log_filename)
                    
#                 if box_center_x > frame_center_x and box_center_y < frame_center_y:
#                      print("bay 41")
#                      bay_xyz(vehicle, 0, 0, -ctr_signal_z, ctr_signal_roll, ctr_signal_pitch, 0, log_filename)
# #                 bay_xyz(vehicle, 0, 0, 0, -ctr_signal_roll, -ctr_signal_pitch, -ctr_signal_z)
#                 if box_center_x >= (frame_center_x-(frame_center_x*0.15)) and box_center_x <= (frame_center_x+(frame_center_x*0.15)):
#                     if box_center_y >= (frame_center_y-(frame_center_y*0.15)) and box_center_y <= (frame_center_y+(frame_center_y*0.15)):
#                         bay_xyz(vehicle, 0, 0, 0, 0, 0, 0,log_filename)
#                         print("target locked")
#                         # time_start = time.time()
#                         # while time.time() - time_start < 5:
#                         time.sleep(1)
#                         bay_xyz(vehicle, 0, 0, 0, 0, 0, 0, log_filename)
#                         break
#                 time.sleep(0.01)
#             cap.release()
#             cv2.destroyAllWindows()
#         ha_canh(vehicle)
        if not var_ref_flag:
            var_ref_flag = True
            i = 0
        if len(thu_tu_tram) == 1:
            thu_tu_tram[0] = thu_tu_tram[i]
        if len(thu_tu_tram)>1:
            i+=1
            if i>=len(thu_tu_tram): break
            thu_tu_tram[0] = thu_tu_tram[i]
#         if not var_ref_flag:
#             var_ref_flag = True
#             i = 0
#         if i<len(thu_tu_tram):
#             i+=1
#             thu_tu_tram[0] = thu_tu_tram[i]
#         else: break
        Start = False
        port += 1
print("Complete all tasks")
