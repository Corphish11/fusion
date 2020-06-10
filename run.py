from huahongzhi.calculate_3d import calculate_3d
from zhaoyu.get_pic_and_coordinate import get_pic_and_coordinate

config_dir = "./zhaoyu/yolov3-tiny-demo.cfg",
weight_dir = "./zhaoyu/backup/yolov3-tiny-demo_1200.weights"
#输入视频文件
video_dir = './zhaoyu/video/pos1.mp4'
#不同的视频，需要不同的时刻文件，来自zhanghaiyang
person_loc_dir = './zhaoyu/track_person_net1.json'
#图片暂存路径
screenshot_dir = './zhaoyu/screenshot/pos1/'
#zhaoyu输出路径
output_dir = '../json/res_pos4_net2_0.json'
get_pic_and_coordinate(person_loc_dir, video_dir, output_dir, screenshot_dir, config_dir, weight_dir)




# #前镜头2d坐标识别结果
# traj_dir_2view_1 = './json/track_result_ball_sbz1.json'
# #后镜头2d坐标识别结果
# traj_dir_2view_2 = './json/track_result_ball_sbz2.json'
# #前端文件输出路径
# output_dir = './json/output.json'
# calculate_3d(traj_dir_2view_1, traj_dir_2view_2, output_dir)