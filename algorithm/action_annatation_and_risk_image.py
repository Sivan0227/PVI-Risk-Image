import constant
import utils
import math
import numpy as np
import shapely.geometry as geo
import cv2


vmax = constant.vmax
v_interval = constant.v_interval
v_num = int(vmax / v_interval)
angle_number = constant.angle_number
time = constant.time
each_angle = (180 / angle_number)
outside_color = constant.outside_color
yielding_color = constant.yielding_color
risky_color = constant.risky_color
safe_color = constant.safe_color
group_color = constant.group_color
leader_color = constant.leader_color
his_color = constant.his_color
block_size = constant.block_size
outside_color = [0, 0, 0]


def paint_motion_pic(boundray):

    x_list = [i * (1 / time) for i in range(-vmax * time, vmax * time + 1, 1)]
    y_list = [i * (1 / time) for i in range(0, vmax * time + 1, 1)]

    pic_arry = np.zeros((vmax * time + 1, 2 * vmax * time + 1, 3))
    for i, v_y in enumerate(y_list):
        for j, v_x in enumerate(x_list):
            if math.sqrt(v_x ** 2 + v_y ** 2) > vmax:
                pic_arry[i][j] = outside_color
            else:
                heading = math.degrees(
                    utils.points_heading((0, 0), (v_x, v_y))) + 90
                heading = heading % 360
                index = math.ceil(heading / each_angle)
                if math.sqrt(v_x ** 2 + v_y ** 2) < boundray[index][0]:
                    pic_arry[i][j] = yielding_color
                elif math.sqrt(v_x ** 2 + v_y ** 2) < boundray[index][1]:
                    pic_arry[i][j] = risky_color
                else:
                    pic_arry[i][j] = safe_color
    return pic_arry


def head(ped_heading, car_heading):
    new = ped_heading - car_heading
    print("ped_heading", ped_heading, car_heading, new)
    if new < 0:
        new += math.radians(360)
    if new > math.radians(180):
        new = math.radians(360) - new
    print("new", new)
    return new


def block(img, loc, color):
    new_num = int((block_size - 1) / 2)
    w = img.shape[1]
    h = img.shape[0]
    left = max(loc[1] - new_num, 0)
    right = min(loc[1] + new_num, w - 1)
    bottom = 0
    up = max(loc[0] - new_num, bottom)
    down = min(loc[0] + new_num, h - 1)
    for m in range(left, right + 1):
        for n in range(up, down + 1):
            img[n][m] = color
    return img


def add_color(heading, v, color, pic_array):
    print(heading, math.sin(heading))
    vx = math.sin(heading) * v
    vy = math.cos(heading) * v
    pic_array = block(
        pic_array, [int(vy / (1 / time)), int((vx + vmax) / (1 / time))], color)
    return pic_array


def color_block(data, seg_num, idx, pic_array):
    true_idx = seg_num-idx
    avg_head = head(data['seg_ped_avg_h'][true_idx],
                    data['seg_car_avg_h'][true_idx])
    group_heading = data['seg_group_avg_h'][true_idx]
    leader_heading = data['seg_leader_avg_h'][true_idx]
    avg_v = data['seg_ped_avg_v'][true_idx]
    group_v = data['seg_group_avg_v'][true_idx]
    leader_v = data['seg_leader_avg_v'][true_idx]
    if data['seg_ped_avg_v'][true_idx] <= vmax:
        avg_head = head(data['seg_ped_avg_h'][true_idx],
                        data['seg_car_avg_h'][true_idx])
        pic_array = add_color(avg_head, avg_v, his_color, pic_array)
    if group_v != np.nan and group_v <= vmax:
        pic_array = add_color(group_heading, group_v, group_color, pic_array)
    if leader_v != np.nan and leader_v <= vmax:
        pic_array = add_color(leader_heading, leader_v,
                              leader_color, pic_array)
    return pic_array


def add_other_info(pic_array, seg_num, data):
    idx1 = 9  # 1s
    idx2 = 4  # 0.5s
    if seg_num >= idx2:
        pic_array = color_block(data, seg_num, idx2, pic_array)
    if seg_num >= idx1:
        pic_array = color_block(data, seg_num, idx1, pic_array)


def determine_status(car_traj_point, ped_traj_point, info, seg_idx):
    traj_num = len(car_traj_point)
    found_boundary = False
    ped_idx = -1
    car_idx = -1
    for traj_idx_i in range(traj_num, -1, -1):
        if traj_idx_i < 2:
            break
        # draw veh traj buffer
        car_traj = car_traj_point[:traj_idx_i]
        car_line = geo.LineString(car_traj)
        veh_polygon = car_line.buffer(0.3 / 100000)
        if not found_boundary:
            for traj_idx_j in range(traj_num, -1, -1):
                if traj_idx_j < 2:
                    continue
                # draw ped traj buffer
                ped_traj = ped_traj_point[:traj_idx_j]
                ped_line = geo.LineString(ped_traj)
                ped_polygon = ped_line.buffer(0.3 / 100000)
                # whether two buffer cross
                if not veh_polygon.intersects(ped_polygon):
                    if traj_idx_j != traj_num:
                        ped_idx = traj_idx_j + 1
                        found_boundary = True
                    break
            if not found_boundary:
                break
        else:
            ped_traj = ped_traj_point[:ped_idx]
            ped_line = geo.LineString(ped_traj)
            ped_polygon = ped_line.buffer(0.3 / 100000)
            # find the last intersection of two buffers
            if not veh_polygon.intersects(ped_polygon):
                car_idx = traj_idx_i + 1
                break
    if ped_idx != -1 and car_idx != -1:
        if ped_idx - car_idx > 0:
            return 'yielding'
        else:
            return 'safe_crossing'
    else:
        heading_from_veh_to_ped = utils.points_heading(
            tuple(car_traj_point[-1]), tuple(ped_traj_point[-1]))
        heading_car = info['seg_car_avg_h'][seg_idx]

        if heading_car > heading_from_veh_to_ped:
            heading_from_veh_to_ped += math.radians(360)
        if info['ped_crossing_dir'] == 'right':
            if heading_from_veh_to_ped - heading_car > math.radians(180):
                return 'safe_crossing'
            else:
                return 'rangxing'
        else:
            if heading_from_veh_to_ped - heading_car > math.radians(180):
                return 'yielding'
            else:
                return 'safe_crossing'


def determine_boundary_v(status_list, angle_idx, yeild_max_v, free_min_v):
    first_0 = -1
    if status_list[0] == 1 and status_list[1] == 0:
        status_list[0] = 0
    for sta_idx in range(len(status_list)):
        if sta_idx > 0 and sta_idx < len(status_list)-1:
            if status_list[sta_idx-1] == 0 and status_list[sta_idx+1] == 0 and status_list[sta_idx] == 2:
                status_list[sta_idx] = 0
    for sta_idx, sta in enumerate(status_list):
        if sta == 1 and yeild_max_v == -1:
            yeild_max_v = (sta_idx + 1) * v_interval
        if sta == 2 and free_min_v == -1:
            free_min_v = (sta_idx + 1) * v_interval
        if sta == 0 and first_0 == -1:
            first_0 = (sta_idx + 1) * v_interval
    if yeild_max_v == -1 and free_min_v == -1:
        yeild_max_v = vmax
        free_min_v = vmax
    else:
        if yeild_max_v != -1:
            yeild_max_v = yeild_max_v
        else:
            if 0 not in status_list:
                free_min_v = 0
                yeild_max_v = 0
            elif angle_idx * (180 / angle_number) > 130:
                if free_min_v < 0.6:
                    free_min_v = v_interval
                    yeild_max_v = v_interval
                else:
                    free_min_v = vmax
                    yeild_max_v = vmax
            else:
                yeild_max_v = free_min_v

        if free_min_v != -1:
            free_min_v = free_min_v
        else:
            if first_0 > yeild_max_v:
                yeild_max_v = vmax
            free_min_v = vmax
    if yeild_max_v > free_min_v:
        # means 2 is followed by 1, with the actual 2 positioned after 1
        if status_list[-1] == 1:
            free_min_v = vmax
        else:
            for idx, value in enumerate(status_list):
                if idx < len(status_list) - 1 and value == 1 and status_list[idx + 1] == 2:
                    free_min_v = (idx + 2) * v_interval
    if yeild_max_v == v_interval:
        yeild_max_v = 0
    if free_min_v == v_interval:
        free_min_v = 0
    return yeild_max_v, free_min_v


def action_label(ped_info):
    seg_num = len(ped_info['seg_num'])
    curr_action_list_all_seg = []
    boundray_list_all_seg = []
    for seg_idx in range(seg_num):
        boundray_dict = {}
        for angle_idx in range(angle_number + 1):
            status_list_same_angle = []
            angle = math.radians(angle_idx * each_angle)
            yeild_max_v = -1  # set intial speed
            free_min_v = -1   # set intial speed

            for v_idx in range(v_num):

                car_traj_point, ped_traj_point = utils.calc_traj_point(
                    (v_idx + 1) * v_interval, seg_idx, ped_info, angle)
                car_traj_heading = utils.calc_traj_heading(car_traj_point)
                car_footprint = utils.calc_motor_footprint(
                    car_traj_point, car_traj_heading)
                overlapped, _ = utils.predict_collision(
                    car_footprint, ped_traj_point)

                # yielding 0 collision 1 safe crossing 2
                if overlapped:
                    status_list_same_angle.append(1)
                else:
                    status = determine_status(
                        car_traj_point, ped_traj_point, ped_info, seg_idx)
                    if status == 'yielding':
                        status_list_same_angle.append(0)
                    else:
                        status_list_same_angle.append(2)
            yeild_max_v, free_min_v = determine_boundary_v(
                status_list_same_angle, angle_idx, yeild_max_v, free_min_v)

            boundray_dict[angle_idx] = [yeild_max_v, free_min_v]

        ped_v = ped_info['seg_ped_avg_v'][seg_idx]
        ped_h = ped_info['seg_ped_avg_h'][seg_idx]
        new_head = head(ped_h, ped_info['seg_car_avg_h'][seg_idx])
        true_angle_index = math.ceil(math.degrees(new_head) / each_angle)
        vx_now = math.sin(new_head) * ped_v
        vy_now = math.cos(new_head) * ped_v
        if math.sqrt(vx_now ** 2 + vy_now ** 2) <= boundray_dict[true_angle_index][0]:
            curr_action_list_all_seg.append(0)
        elif math.sqrt(vx_now ** 2 + vy_now ** 2) <= boundray_dict[true_angle_index][1]:
            curr_action_list_all_seg.append(1)
        else:
            curr_action_list_all_seg.append(2)
        boundray_list_all_seg.append(boundray_dict)
    return curr_action_list_all_seg, boundray_list_all_seg


def generate_motion_pic(boundray_list_all_seg, ped_info):
    for seg, boundray in enumerate(boundray_list_all_seg):
        pic_arry = paint_motion_pic(boundray)
        add_other_info(pic_arry, seg, ped_info)
        cv2.imwrite('./algorithm/risk_image/'+str(seg) + '.png', pic_arry)


if __name__ == '__main__':

    one_ped_info = {
        'ped_id': '1',
        'seg_num': [0, 1, 2, 3, 4],
        'cross': True,
        'ped_crossing_dir': 'left',
        'seg_car_loc': [[-81.54927944655948, 6.429980591089609], [-82.31095461250264, 6.54679649674744],
                        [-83.05742509035734, 6.665760834569962], [-83.79004325016825, 6.7848213101814405],
                        [-84.5055393896481, 6.905313711685152], [-85.20370989723078, 7.027244145888625],],
        'seg_ped_loc': [[-118.07308000464937, 9.152431511633333], [-118.02624195834309, 9.270064183087667],
                        [-117.98254823995275,
                            9.386253133805052], [-117.96545996722396, 9.533519183036312],
                        [-117.9367517692744, 9.6578454969176], [-117.90721372574974, 9.787760512366372]],
        'seg_car_avg_v': [7.40508019915635, 7.255131200030104, 7.117755402468026, 6.987626386500061,
                          6.874974508718057, 6.767980599500102],
        'seg_car_avg_vx': [-7.308000000000002, -7.153999999999999, -7.014, -6.882000000000001,
                           -6.767999999999999, -6.659999999999999],
        'seg_car_avg_vy': [1.194, 1.206, 1.21, 1.2099999999999997, 1.2079999999999997, 1.2040000000000002],
        'seg_ped_avg_v': [1.319645201603826, 1.3402480097937017, 1.370234095178267, 1.35630300586297,
                          1.3875550855147316, 1.4086245655944396],
        'seg_ped_avg_vx': [0.33399999999999996, 0.3, 0.27, 0.294, 0.294, 0.292],
        'seg_ped_avg_vy': [1.27, 1.302, 1.342, 1.324, 1.3559999999999999, 1.3780000000000001],
        'seg_car_avg_a': [-1.4979999999999998, -1.3719999999999999, -1.3, -1.126, -1.07, -1.0859999999999999],
        'seg_car_avg_ax': [1.54, 1.4, 1.32, 1.1400000000000001, 1.08, 1.1],
        'seg_car_avg_ay': [0.12, 0.04, 0.0, -0.019999999999999997, -0.04000000000000001, -0.020000000000000007],
        'seg_ped_avg_a': [0.20600000000000004, 0.3, -0.13799999999999998, 0.31399999999999995,
                          0.21200000000000002, 0.17400000000000002],
        'seg_ped_avg_ax': [-0.33999999999999997, -0.30000000000000004, 0.24, 0.0, -0.02, -0.02],
        'seg_ped_avg_ay': [0.32, 0.4000000000000001, -0.18, 0.31999999999999995, 0.22000000000000003, 
                           0.18000000000000002],
        'seg_car_avg_h': [4.8740000000000006, 4.880000000000001, 4.884, 4.888, 4.89, 4.892],
        'seg_ped_avg_h': [0.262, 0.22999999999999998, 0.19999999999999998, 0.21800000000000003, 0.21400000000000002,
                          0.21000000000000002],
        'seg_group_avg_v': [2, 2, 2.2, 2, 2.2, 2.1],
        'seg_group_avg_h': [0, 0, 0, 0, 0],
        'seg_leader_avg_v': [3, 3, 3.2, 3, 3.2, 3.1],
        'seg_leader_avg_h': [0, 0, 0, 0, 0],
    }
    curr_action_list_all_seg, boundray_list_all_seg = action_label(one_ped_info)
    """

    curr_action_list_all_seg = [
    [0,0,0,0,1,1,1,1,2,2,2,2...],[],[],...
    ]

    boundray_list_all_seg = [
    {
    '0':[v1,v2],'1':[,],...,'30':[,]
    },
    ...
    ]
    
    """

    generate_motion_pic(boundray_list_all_seg, one_ped_info)
