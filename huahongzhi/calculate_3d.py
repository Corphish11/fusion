import numpy as np
from scipy.linalg import svd
import cv2
from huahongzhi.solve_pnp import DLT
import pickle
import json
import math


def build_matrix(tracklet_2d, g, P, times):
    M = []
    n = len(times)
    R = []
    start_time = times[0]
    for i in range(n):
        frame_num, x, y = tracklet_2d[i]
        t = (times[i] - start_time) / 50
        M.append([P[0, 0] - x * P[2, 0], P[0, 0] * t - x * P[2, 0] * t,
                  P[0, 1] - x * P[2, 1], P[0, 1] * t - x * P[2, 1] * t,
                  P[0, 2] - x * P[2, 2], P[0, 2] * t - x * P[2, 2] * t
                  ])
        M.append([P[1, 0] - y * P[2, 0], P[1, 0] * t - y * P[2, 0] * t,
                  P[1, 1] - y * P[2, 1], P[1, 1] * t - y * P[2, 1] * t,
                  P[1, 2] - y * P[2, 2], P[1, 2] * t - y * P[2, 2] * t
                  ])

        R.append(
            [
                x * (P[2, 2] * g * t ** 2 / 2 + 1) - (P[0, 2] * g * t ** 2 / 2 + P[0, 3])
            ]
        )

        R.append(
            [y * (P[2, 2] * g * t ** 2 / 2 + 1) - (P[1, 2] * g * t ** 2 / 2 + P[1, 3])]
        )
    return M, R

def build_matrix_2view(tracklet_2ds_1, tracklet_2ds_2, g, P0, P, times):
    M = []
    n = len(times)
    R = []
    start_time = times[0]
    for i in range(n):
        frame_num1, x0, y0 = tracklet_2ds_1[i]
        t = (times[i] - start_time) / 50
        M.append([P0[0, 0] - x0 * P0[2, 0], P0[0, 0] * t - x0 * P0[2, 0] * t,
                  P0[0, 1] - x0 * P0[2, 1], P0[0, 1] * t - x0 * P0[2, 1] * t,
                  P0[0, 2] - x0 * P0[2, 2], P0[0, 2] * t - x0 * P0[2, 2] * t
                  ])
        M.append([P0[1, 0] - y0 * P0[2, 0], P0[1, 0] * t - y0 * P0[2, 0] * t,
                  P0[1, 1] - y0 * P0[2, 1], P0[1, 1] * t - y0 * P0[2, 1] * t,
                  P0[1, 2] - y0 * P0[2, 2], P0[1, 2] * t - y0 * P0[2, 2] * t
                  ])

        R.append(
            [x0 * (P0[2, 2] * g * t ** 2 / 2 + 1) - (P0[0, 2] * g * t ** 2 / 2 + P0[0, 3])]
        )
        R.append(
            [y0 * (P0[2, 2] * g * t ** 2 / 2 + 1) - (P0[1, 2] * g * t ** 2 / 2 + P0[1, 3])]        
        )
    for i in range(n):
        frame_num2, x, y = tracklet_2ds_2[i]
        t = (times[i] - start_time) / 50
        M.append([P[0, 0] - x * P[2, 0], P[0, 0] * t - x * P[2, 0] * t,
                  P[0, 1] - x * P[2, 1], P[0, 1] * t - x * P[2, 1] * t,
                  P[0, 2] - x * P[2, 2], P[0, 2] * t - x * P[2, 2] * t
                  ])
        M.append([P[1, 0] - y * P[2, 0], P[1, 0] * t - y * P[2, 0] * t,
                  P[1, 1] - y * P[2, 1], P[1, 1] * t - y * P[2, 1] * t,
                  P[1, 2] - y * P[2, 2], P[1, 2] * t - y * P[2, 2] * t
                  ])
        R.append(
            [
                x * (P[2, 2] * g * t ** 2 / 2 + 1) - (P[0, 2] * g * t ** 2 / 2 + P[0, 3])
            ]
        )

        R.append(
            [y * (P[2, 2] * g * t ** 2 / 2 + 1) - (P[1, 2] * g * t ** 2 / 2 + P[1, 3])]
        )
    return M, R


def svd_solve(A, b):
    A = np.asarray(A, dtype=np.float32)
    b = np.asarray(b, dtype=np.float32)
    S, U, V = cv2.SVDecomp(A)
    B = U.T.dot(b)
    X = [B[i]/S[i] for i in range(len(S))]
    res = V.T.dot(X)
    return res

def convert_tracklet_2d_to_3d(start_p, velocity, times):

    pos_3ds = list()
    X0, Y0, Z0 = start_p
    Vx, Vy, Vz = velocity
    start_time = times[0]
    g = -980 #重力加速度

    #可以根据每一帧的相对时间，算出每一帧球的位置
    for time in times:
        delat_time = (time - start_time) / 50 #这里用的当前帧数-起始帧数，算出来实际的秒数
        pos_3ds.append(
            [time,
             float((X0 + delat_time * Vx)/100),
             float((Y0 + delat_time * Vy)/100),
             float((Z0 + delat_time * Vz + g * delat_time ** 2 / 2)/100),
             ]
        )
    return pos_3ds

def multi_traj_process_2view_withp(traj_dir1, traj_dir2, parabola_dir):
    pinyin_json_list = list()
    with open(parabola_dir,'r') as f1:
        parabola_info = json.load(f1) 
    with open(traj_dir1, 'rb') as f2: 
        new_json_data = json.load(f2) 
    with open(traj_dir2, 'rb') as f3: 
        new_json_data2 = json.load(f3)

    new_data = list()
    new_data2 = list()
    for key in new_json_data:
        for item in new_json_data[key]:
            new_data.append([key, item[0], (item[1][0] + item[1][2]/2), (item[1][1] + item[1][3]/2)])
    for key in new_json_data2:
        for item in new_json_data2[key]:
            new_data2.append([key, item[0], (item[1][0] + item[1][2]/2), (item[1][1] + item[1][3]/2)])
    i = 0
    for parabola in parabola_info:
        start_frame = 0
        end_frame = 0
        tracklet_2ds_1 = list()
        tracklet_2ds_2 = list()
        untrimed_tracklet_1 = list()
        untrimed_tracklet_2 = list()
        time_gap = parabola["end_frame_1"] - parabola["start_frame_1"]
        time_dislocate = parabola["start_frame_1"] - parabola["start_frame_2"]
        times = list()  
        for item in new_data:
            if item[1] <= parabola["end_frame_1"]:
                if (item[0] == parabola["ball_id_1"]) & (item[1] >= parabola["start_frame_1"]): 
                    untrimed_tracklet_1.append(item)   
            else:
                break
        for item in new_data2:
            if item[1] <= time_gap + parabola["start_frame_2"]:
                if (item[0] == parabola["ball_id_2"]) & (item[1] >= parabola["start_frame_2"]): 
                    untrimed_tracklet_2.append(item)   
            else:
                break
        x = 0
        y = 0
        while (x < len(untrimed_tracklet_1)) & (y < len(untrimed_tracklet_2)):
            if untrimed_tracklet_1[x][1] == (untrimed_tracklet_2[y][1] + time_dislocate):
                tracklet_2ds_1.append([untrimed_tracklet_1[x][0], untrimed_tracklet_1[x][2], untrimed_tracklet_1[x][3]])
                tracklet_2ds_2.append([untrimed_tracklet_2[y][0], untrimed_tracklet_2[y][2], untrimed_tracklet_2[y][3]])
                times.append(untrimed_tracklet_1[x][1])
                x += 1
                y += 1
            elif untrimed_tracklet_1[x][1] < untrimed_tracklet_2[y][1] + time_dislocate:
                x += 1
            elif untrimed_tracklet_1[x][1] > untrimed_tracklet_2[y][1] + time_dislocate:
                y += 1
        print(times)
        print(len(times))
        print('\n')
        g = -980
        P0, P = DLT()
        A, b = build_matrix_2view(tracklet_2ds_1, tracklet_2ds_2, g, P0, P, times)
        X0, Vx, Y0, Vy, Z0, Vz = svd_solve(A, b)
        path = list()
        for item in tracklet_2ds_1:
            path.append(round((item[1]/1920),3))
            path.append(round((item[2]/1080),3))

        V_init = [round(float(Vx/100*3.6), 3), round(float(Vy/100*3.6), 3), round(float(Vz/100*3.6), 3)]
        XYZ_init = [round(float(X0/100), 3), round(float(Y0/100), 3), round(float(Z0/100), 3)]
        ball_velocity = round(math.sqrt(V_init[0] ** 2 + V_init[1] ** 2 + V_init[2] ** 2), 3)
        start_frame = times[0]
        end_frame = parabola["end_frame_1"]
        start_time = round((start_frame / 50), 2)
        end_time = round((end_frame / 50), 2)
        pinyin_json_list.append({"ks":start_time, "js":end_time, "qy":"未定义", "dz":"二传", "zdz":"", "init_pos":XYZ_init, "velocity":V_init, "qs":ball_velocity, "path":path})
        pinyin_dump = {"jdlist":pinyin_json_list}
        with open("./output/sbz.json","w") as f:
            json.dump(pinyin_dump,f,ensure_ascii=False)

def multi_traj_process_2view(traj_dir1, traj_dir2, output_dir):
    pinyin_json_list = list()
    with open(traj_dir1, 'rb') as f2: 
        new_json_data = json.load(f2) 
    with open(traj_dir2, 'rb') as f3: 
        new_json_data2 = json.load(f3)
    #frame max number
    threshold_upper = 45
    #frame min number
    threshold_lower = 20
    #start frame gap
    threshold2 = 20
    for key in new_json_data:
        untrimed_tracklet_1 = list()
        untrimed_tracklet_2 = list()
        if (len(new_json_data[key]) > threshold_lower) and (len(new_json_data[key]) < threshold_upper):
            for key2 in new_json_data2:
                if (abs(new_json_data2[key2][0][0] - new_json_data[key][0][0]) < threshold2) and (len(new_json_data2[key2]) > threshold_lower) and (len(new_json_data2[key2]) < threshold_upper):
                    for item in new_json_data[key]:
                        untrimed_tracklet_1.append([key, item[0], (item[1][0] + item[1][2]/2), (item[1][1] + item[1][3]/2)])
                    for item in new_json_data2[key2]:
                        untrimed_tracklet_2.append([key, item[0], (item[1][0] + item[1][2]/2), (item[1][1] + item[1][3]/2)])
                    x = 0
                    y = 0
                    time_dislocate = untrimed_tracklet_1[0][1] - untrimed_tracklet_2[0][1]
                    tracklet_2ds_1 = list()
                    tracklet_2ds_2 = list()
                    times = list()
                    while (x < len(untrimed_tracklet_1)) & (y < len(untrimed_tracklet_2)):
                        if untrimed_tracklet_1[x][1] == (untrimed_tracklet_2[y][1] + time_dislocate):
                            tracklet_2ds_1.append([untrimed_tracklet_1[x][0], untrimed_tracklet_1[x][2], untrimed_tracklet_1[x][3]])
                            tracklet_2ds_2.append([untrimed_tracklet_2[y][0], untrimed_tracklet_2[y][2], untrimed_tracklet_2[y][3]])
                            times.append(untrimed_tracklet_1[x][1])
                            x += 1
                            y += 1
                        elif untrimed_tracklet_1[x][1] < untrimed_tracklet_2[y][1] + time_dislocate:
                            x += 1
                        elif untrimed_tracklet_1[x][1] > untrimed_tracklet_2[y][1] + time_dislocate:
                            y += 1
                    print(times)
                    print(len(times))
                    print('\n')
                    g = -980
                    P0, P = DLT()
                    A, b = build_matrix_2view(tracklet_2ds_1, tracklet_2ds_2, g, P0, P, times)
                    X0, Vx, Y0, Vy, Z0, Vz = svd_solve(A, b)
                    path = list()
                    for item in tracklet_2ds_1:
                        path.append(round((item[1]/1920),3))
                        path.append(round((item[2]/1080),3))

                    V_init = [round(float(Vx/100*3.6), 3), round(float(Vy/100*3.6), 3), round(float(Vz/100*3.6), 3)]
                    XYZ_init = [round(float(X0/100), 3), round(float(Y0/100), 3), round(float(Z0/100), 3)]
                    ball_velocity = round(math.sqrt(V_init[0] ** 2 + V_init[1] ** 2 + V_init[2] ** 2), 3)
                    start_frame = times[0]
                    end_frame = times[-1]
                    start_time = round((start_frame / 50), 2)
                    end_time = round((end_frame / 50), 2)
                    pinyin_json_list.append({"ks":start_time, "js":end_time, "qy":"未定义", "dz":"二传", "zdz":"", "init_pos":XYZ_init, "velocity":V_init, "qs":ball_velocity, "path":path})
                    pinyin_dump = {"jdlist":pinyin_json_list}
                    with open(output_dir,"w") as f:
                        json.dump(pinyin_dump,f,ensure_ascii=False)

def calculate_3d(traj_dir_2view_1, traj_dir_2view_2, output_dir):

    multi_traj_process_2view(traj_dir_2view_1, traj_dir_2view_2, output_dir)
