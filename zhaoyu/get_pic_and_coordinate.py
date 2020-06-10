# -*- coding:utf-8 -*-

"""
      ┏┛ ┻━━━━━┛ ┻┓
      ┃　　　　　　 ┃
      ┃　　　━　　　┃
      ┃　┳┛　  ┗┳　┃
      ┃　　　　　　 ┃
      ┃　　　┻　　　┃
      ┃　　　　　　 ┃
      ┗━┓　　　┏━━━┛
        ┃　　　┃   神兽保佑
        ┃　　　┃   代码无BUG！
        ┃　　　┗━━━━━━━━━┓
        ┃　　　　　　　    ┣┓
        ┃　　　　         ┏┛
        ┗━┓ ┓ ┏━━━┳ ┓ ┏━┛
          ┃ ┫ ┫   ┃ ┫ ┫
          ┗━┻━┛   ┗━┻━┛
"""

import os
import json
from tqdm import tqdm

from zhaoyu.screenshot_tools import video2frames

from zhaoyu.find_coordinate import *
# from my_code.print_pic import *
from zhaoyu.find_coordinate import my_performDetect, get_coor

MIN_DISTANCE = 50
DISAPPEAR_ITER = 50


def get_pic(pathOut, file_name='./test1.mp4'):
    """
    获取视频图片
    :param file_name:
    :return:
    """
    video2frames(pathIn=file_name, pathOut=pathOut,  # initial_extract_time=6, end_extract_time=8,
                 output_prefix='pos4')


def del_pic(file_name):
    """
    隔一帧删除一个画面，减少计算（弃用）
    :return:
    """
    flag = False
    for each in os.listdir(file_name):
        if flag is False:
            flag = True
            continue
        os.remove(file_name + '/' + each)
        flag = False
        print(file_name + '/' + each)


# get_pic(file_name='../video/pos4.mp4', pathOut='../screenshot/pos4/')


# del_pic('../screenshot/pos1/')


def get_coordinate(file_path, config_path, weight_path):
    """
    获取坐标
    :return:
    """
    netMain, metaMain = my_performDetect(configPath=config_path, weightPath=weight_path)
    res = []
    # file_path = '../screenshot/test/'
    file_list = os.listdir(file_path)
    # len(file_list)
    for i in tqdm(range(len(file_list))):
        each = file_list[i]
        if each.endswith('txt'):
            continue
        path = file_path + each
        detections = get_coor(netMain=netMain, metaMain=metaMain, imagePath=path, thresh=0.5, makeImageOnly=True)
        res.append(detections['detections'])
    with open('./json/res_2.txt', 'w') as f:
        f.write(json.dumps(res))
    return res


# get_coordinate()


def read_coordinate():
    """
    从文件中读取坐标
    :return:
    """
    with open('./json/res_2.txt', 'r') as f:
        res = f.read()
    res_list = json.loads(res)
    return res_list


def find_neareat_pos(tar_pos_list, base_pos_list):
    """
    寻找最近的距离
    :return:
    """
    # 用来记录当前插入状态，为没有插入的轨迹添加与[-1]一样的坐标
    if len(base_pos_list) >= 130:
        print()
        pass
    add_flag = [False for _ in range(len(base_pos_list))]
    for i in range(len(tar_pos_list)):
        each_tar = tar_pos_list[i]
        euclid_dis = []
        for each_base in base_pos_list:
            euclid_dis.append(compute_euclid(each_tar, each_base[-1]))
        # 判断目标球是否是新检测出来的求，判断方法是和上一帧欧氏距离差距MIN DISTANCE以上
        print(len(add_flag))
        if min(euclid_dis) < MIN_DISTANCE:
            min_idx = euclid_dis.index(min(euclid_dis))
            base_pos_list[min_idx].append(each_tar)
            add_flag[min_idx] = True
        else:
            # 新球
            base_pos_list.append([each_tar] * len(add_flag))
            add_flag.append(True)
    # 处理球丢失跟踪的情况，将坐标重复放置
    for i in range(len(add_flag)):
        if add_flag[i] is False:
            base_pos_list[i].append(base_pos_list[i][-1])
        pass
    return base_pos_list
    pass


def compute_center_point(pos_list):
    """
    根据左上角点的x y  物体的长宽 计算物体中心点
    :param x:
    :param y:
    :param h:
    :param w:
    :return:
    """
    x, y, h, w = pos_list
    return x + 2 / w, y + 2 / w


def compute_euclid(tar_pos, base_pos):
    """
    计算欧几里得距离
    :param tar_pos:
    :param base_pos:
    :return:
    """
    return (float(tar_pos[0]) - float(base_pos[0])) ** 2 + (float(tar_pos[1]) - float(base_pos[1])) ** 2
    pass


def find_pos(res_list):
    """
    寻找轨迹
    :param res_list:
    :return:
    """
    ball_path = []
    for i in range(len(res_list[0])):
        ball_path.append([res_list[0][i]])
    for i in range(1, len(res_list)):
        ball_path = find_neareat_pos(res_list[i], ball_path)

        pass
    return ball_path
    pass


def handle_data(res_list):
    """
    处理数据格式
    :param res_list:
    :return:
    """
    handled_data = []
    for i in range(len(res_list)):
        temp = []
        for j in range(len(res_list[i])):
            temp.append(compute_center_point(res_list[i][j][2]))
        handled_data.append(temp)
    return handled_data


def find_coordinate_v2(res_list, file_list):
    """
    寻找坐标
    :param res_list:
    :return:
    """
    res = []
    idx = 1
    last_handle = 0
    for i in range(len(res_list)):
        for j in range(len(res_list[i])):
            if 950 < res_list[i][j][0] < 1000:
                if i - last_handle > 10:
                    last_handle = i
                    res.append([i, [res_list[i][j][0], res_list[i][j][1]]])
                    pass
    # for each in res:
    #     print(file_list[each])
    # print(len(res))
    return res


def find_coordinate_v3(res_list, person_loc):
    """
    寻找坐标
    :param res_list:
    :return:
    """
    res = []
    idx = 1
    last_handle = 0
    # for i in range(len(person_loc)):
    #     print(person_loc[i])
    # for k, v in person_loc:
    #     print(k, v)
    for i in range(len(res_list)):
        if i == 0:
            continue
        temp = []
        need_pos = person_loc[str(i + 1)]
        bx, _, bw, _ = need_pos
        bw = bx + bw + 15
        bx -= 15
        for j in range(len(res_list[i])):
            if bx < float(res_list[i][j][0]) < bw and float(res_list[i][j][1]) < 380:
                # if 750 < float(res_list[i][j][0]) < 1200 and 320 < float(res_list[i][j][1]) < 380:
                x, y, w, h = float(res_list[i][j][0]), float(res_list[i][j][1]), float(res_list[i][j][2]), float(
                    res_list[i][j][3])
                flag = False
                for k in range(len(temp)):
                    x_t, y_t, w_t, h_t = temp[k]
                    if -30 < x_t - x < 30 and -30 < y_t - y < 30:
                        flag = True
                        break
                if flag:
                    continue
                temp.append([x, y, w, h])
                res.append([i, [float(res_list[i][j][0]) - float(res_list[i][j][2]) / 2,
                                float(res_list[i][j][1]) - float(res_list[i][j][3]) / 2, float(res_list[i][j][2]),
                                float(res_list[i][j][3])]])

    # for each in res:
    #     print(file_list[each])
    # print(len(res))
    return res


def find_coordinate_v4(res_list, person_loc):
    """
    寻找坐标
    :param res_list:
    :return:
    """
    res = []
    idx = 1

    # for i in range(len(person_loc)):
    #     print(person_loc[i])
    # for k, v in person_loc:
    #     print(k, v)
    for i in range(len(res_list)):
        if i == 0:
            continue
        temp = []
        need_pos = person_loc[str(i + 1)]
        bx, _, bw, _ = need_pos
        bw = bx + bw + 15
        bx -= 15
        for j in range(len(res_list[i])):
            if bx < float(res_list[i][j][0]) < bw and float(res_list[i][j][1]) < 380:
                # if 750 < float(res_list[i][j][0]) < 1200 and 320 < float(res_list[i][j][1]) < 380:
                x, y, w, h = float(res_list[i][j][0]), float(res_list[i][j][1]), float(res_list[i][j][2]), float(
                    res_list[i][j][3])
                flag = False
                for k in range(len(temp)):
                    x_t, y_t, w_t, h_t = temp[k]
                    if -30 < x_t - x < 30 and -30 < y_t - y < 30:
                        flag = True
                        break
                if flag:
                    continue
                temp.append([x, y, w, h])
                res.append([i, [float(res_list[i][j][0]) - float(res_list[i][j][2]) / 2,
                                float(res_list[i][j][1]) - float(res_list[i][j][3]) / 2, float(res_list[i][j][2]),
                                float(res_list[i][j][3])]])

    res_final = []
    last_handle = -1
    save_data = [0, 0, 0, 0]
    save_idx = -1
    for i in range(len(res)):
        x, coor = res[i]
        if x - last_handle == 1 or last_handle == -1:
            last_handle = x
            if coor[1] > save_data[1]:
                save_data = coor
                save_idx = x
            pass
        else:
            res_final.append([save_idx, save_data])
            save_data = [0, 0, 0, 0]
            last_handle = -1
    res_final.append(save_data)

    return res_final


def get_files():
    res = []
    for each in os.listdir('../screenshot/pos4/'):
        if each.endswith('txt'):
            continue
        res.append(each)
        # print('data/obj/' + each)
    return res


def read_person_loc(person_loc_dir):
    """
    从文件中读取任务坐标
    :return:
    """
    with open(person_loc_dir, 'r') as f:
        res = f.read()
    res_list = json.loads(res)
    return res_list


def get_pic_and_coordinate(person_loc_dir, video_dir, output_dir, screenshot_dir, config_path, weight_path):
    get_pic(file_name=video_dir, pathOut=screenshot_dir)
    # get_files()
    get_coordinate(file_path=screenshot_dir, config_path=config_path, weight_path=weight_path)
    res_list = read_coordinate()
    res_handled_list = []
    for i in range(len(res_list)):
        temp = []
        for j in range(len(res_list[i])):
            temp.append(res_list[i][j][2])
        res_handled_list.append(temp)
        pass
    # res_list = handle_data(res_list)
    # file_list = get_files()
    person_loc = read_person_loc(person_loc_dir)
    ball_frame = find_coordinate_v4(res_handled_list, person_loc)
    print(ball_frame)

    with open(output_dir, 'w+') as r:
        r.write(json.dumps(ball_frame))
    # ball_path = find_pos(res_list)
    # for i in range()
    pass
    # ['bv', 0.6526369452476501, [917.3140258789062, 142.1123046875, 29.449554443359375, 35.156471252441406]]
    # ['bv', 0.8256919980049133, [969.90771484375, 145.88124084472656, 33.613609313964844, 32.712581634521484]]
