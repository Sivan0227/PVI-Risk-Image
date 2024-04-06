import numpy as np
import scipy.linalg as lg
import math
import shapely.geometry as geo
import constant

predict_duration = constant.predict_duration
predict_interval = constant.predict_interval
num_of_traj_points = int(predict_duration / predict_interval)
car_w, car_l, ped_rad = constant.car_w, constant.car_l, constant.ped_rad


def differentiate(x: list, t: list) -> list:
    """Difference-Based Derivative Calculation Method."""
    if len(x) > len(t):
        raise ValueError("length mismatched")
    if len(x) <= 1:
        raise ValueError("length of x must be greater than 1")
    return [round((x[i + 1] - x[i]) / (t[i + 1] - t[i]), 2) for i in range(len(x) - 1)]


def mean(x: list) -> float:
    """Mean calculation method."""
    if len(x) == 0:
        raise ValueError("x cannot be empty")
    return sum(x) / len(x)


def cv_motion_point(p0: tuple, v: tuple, t: float) -> tuple:
    """Trajectory calculation for constant velocity model."""
    return tuple(p0[i] + v[i] * t for i in range(len(p0)))


def ca_motion_point(p0: tuple, v: tuple, a: tuple, t: float) -> tuple:
    """Trajectory calculation for constant acceleration model."""
    point = []
    zero = False
    for i in range(2):
        if (a[i] * t + v[i]) * v[i] < 0:
            point.append(p0[i] - (v[i] ** 2 / (2 * a[i])))
            zero = True
        else:
            point.append(p0[i] + v[i] * t + 0.5 * a[i] * t * t)
    if zero:
        point = []
        for i in range(2):
            point.append(p0[i] - (v[i] ** 2 / (2 * a[i])))
    return tuple(point)

    # return tuple(p0[i] + v[i] * t + 0.5 * a[i] * t * t for i in range(len(p0)))


def normalize_radians(r0: float) -> float:
    """Normalize heading between 0 - 2 pi."""
    return r0 % (math.pi * 2)


def points_heading(p0: tuple, p1: tuple) -> float:
    """Heading calculation by two points."""
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    theta = math.pi / 2 - math.atan2(dy, dx)
    if theta < 0:
        return theta + 2 * math.pi
    else:
        return theta


def points_heading_array(p0x, p0y, p1x, p1y) -> float:
    """Heading calculation by two points."""
    dx = p1x - p0x
    dy = p1y - p0y
    theta = math.pi / 2 - lg.tanhm(dy, dx)
    theta[np.where(theta < 0)] = theta[np.where(theta < 0)] + 2 * math.pi
    return theta


def build_rectangle(
        point: tuple, heading: float, length: float, width: float
) -> tuple:
    """Calculate the corner coordinates of a rectangle at any angle."""
    x = point[0]
    y = point[1]

    x0 = x - 0.5 * width
    y0 = y + 0.5 * length

    x1 = x + 0.5 * width
    y1 = y0

    x2 = x1
    y2 = y - 0.5 * length

    x3 = x0
    y3 = y2

    x0, y0 = rotation(x0 - x, y0 - y, heading)
    x1, y1 = rotation(x1 - x, y1 - y, heading)
    x2, y2 = rotation(x2 - x, y2 - y, heading)
    x3, y3 = rotation(x3 - x, y3 - y, heading)

    x0 = x0 + x
    x1 = x1 + x
    x2 = x2 + x
    x3 = x3 + x
    y0 = y0 + y
    y1 = y1 + y
    y2 = y2 + y
    y3 = y3 + y

    return (x0, y0), (x1, y1), (x2, y2), (x3, y3)


def rotation(x: float, y: float, theta: float) -> tuple:
    """Rotation function between coordinate systems."""
    cos = math.sin(theta)
    sin = math.cos(theta)
    return x * cos - y * sin, x * sin + y * cos


def is_rects_overlapped(rect0: tuple, rect1: tuple) -> bool:
    """Determine if two rectangles overlap."""
    rect0_geo = geo.Polygon(rect0)
    rect1_geo = geo.Polygon(rect1)
    return rect0_geo.intersects(rect1_geo)


def is_rect_and_circle_overlapped(
        rect: tuple, point: tuple, radius: float
) -> bool:
    """Determine if rectangle and circle overlap."""
    rect_geo = geo.Polygon(rect)
    point_buffer = geo.Point(point).buffer(radius)
    return point_buffer.intersects(rect_geo)


def calcu_head(seg_num, kinematics, heading):
    if kinematics['ped_crossing_dir'] == 'left':
        heading = kinematics['seg_car_avg_h'][seg_num] + heading
        heading = heading if heading < math.radians(
            360) else heading - math.radians(360)
    else:
        heading = kinematics['seg_car_avg_h'][seg_num] - heading
        heading = heading if heading < math.radians(
            0) else heading + math.radians(360)
    return heading


def calc_traj_point_CA(seg_num, kinematics):
    num_of_traj_points = int(predict_duration / predict_interval)
    # 恒定加速度模型
    car_loc = kinematics['seg_car_loc'][seg_num]
    ped_loc = kinematics['seg_ped_loc'][seg_num]

    car_vx = kinematics['seg_car_avg_vx'][seg_num]
    car_vy = kinematics['seg_car_avg_vy'][seg_num]

    car_ax = kinematics['seg_car_avg_ax'][seg_num]
    car_ay = kinematics['seg_car_avg_ay'][seg_num]

    ped_vx = kinematics['seg_ped_avg_vx'][seg_num]
    ped_vy = kinematics['seg_ped_avg_vy'][seg_num]

    ped_ax = kinematics['seg_ped_avg_ax'][seg_num]
    ped_ay = kinematics['seg_ped_avg_ay'][seg_num]

    # 未来轨迹点信息为列表嵌套轨迹点元组
    car_traj_point = [
        ca_motion_point(
            tuple(car_loc),
            (car_vx, car_vy),
            (car_ax, car_ay),
            predict_interval * (i + 1),
        )
        for i in range(num_of_traj_points)
    ]
    ped_traj_point = [
        ca_motion_point(
            tuple(ped_loc),
            (ped_vx, ped_vy),
            (ped_ax, ped_ay),
            predict_interval * (i + 1),
        )
        for i in range(num_of_traj_points)
    ]

    return car_traj_point, ped_traj_point


def calc_traj_point_CV(seg_num, kinematics):
    # 恒定速度模型
    car_loc = kinematics['seg_car_loc'][seg_num]
    ped_loc = kinematics['seg_ped_loc'][seg_num]

    car_vx = kinematics['seg_car_avg_vx'][seg_num]
    car_vy = kinematics['seg_car_avg_vy'][seg_num]

    ped_vx = kinematics['seg_ped_avg_vx'][seg_num]
    ped_vy = kinematics['seg_ped_avg_vy'][seg_num]

    a = math.sqrt(kinematics['seg_car_avg_ax'][seg_num]
                  ** 2 + kinematics['seg_car_avg_ay'][seg_num] ** 2)

    if kinematics['seg_car_avg_a'][seg_num] < 0:
        a = -abs(a)

    car_ax = a * math.sin(kinematics['seg_car_avg_h'][seg_num])
    car_ay = a * math.cos(kinematics['seg_car_avg_h'][seg_num])

    car_traj_point = [
        ca_motion_point(tuple(car_loc), (car_vx, car_vy),
                        (car_ax, car_ay), predict_interval * (i + 1))
        for i in range(num_of_traj_points)
    ]
    ped_traj_point = [
        cv_motion_point(tuple(ped_loc), (ped_vx, ped_vy),
                        predict_interval * (i + 1))
        for i in range(num_of_traj_points)
    ]

    return car_traj_point, ped_traj_point


def calc_motor_footprint(traj, heading):
    # 足迹即车辆的矩形框位置信息
    footprint = [
        build_rectangle(traj[i], heading[i], car_w, car_l, ) for i in range(num_of_traj_points)
    ]
    return footprint


def calc_traj_heading(traj_point):
    traj_heading = [
        points_heading(traj_point[i], traj_point[i + 1]) for i in range(num_of_traj_points - 1)
    ]
    traj_heading.append(traj_heading[-1])
    return traj_heading


def predict_collision(motor_footprint, vptc_point):
    motor_fp = motor_footprint
    overlapped = False
    for k in range(num_of_traj_points):
        if is_rect_and_circle_overlapped(motor_fp[k], vptc_point[k], ped_rad):
            # print(k)
            # print(motor_fp[k])
            # print(vptc_point[k])
            # print(11111,motor_fp[:k+1])
            # print(22222,vptc_point[:k+1])
            overlapped = True
            break
    if overlapped:
        TTC = (k + 1) * predict_interval
        return overlapped, TTC
    return overlapped, -1


def collision_point(motor_footprint, vptc_point):
    motor_fp = motor_footprint
    fisrt = []
    last = []
    for k in range(num_of_traj_points):
        if is_rect_and_circle_overlapped(motor_fp[k], vptc_point[k], ped_rad):
            # print(k)
            # print(motor_fp[k])
            # print(vptc_point[k])
            # print(11111,motor_fp[:k+1])
            # print(22222,vptc_point[:k+1])
            if not fisrt:
                fisrt = [k, motor_fp[k], vptc_point[k]]
            last = [k, motor_fp[k], vptc_point[k]]
    return fisrt, fisrt


def calc_traj_point(v, seg_num, kinematics, head):
    car_loc = kinematics['seg_car_loc'][seg_num]
    ped_loc = kinematics['seg_ped_loc'][seg_num]

    car_vx = kinematics['seg_car_avg_vx'][seg_num]
    car_vy = kinematics['seg_car_avg_vy'][seg_num]

    a = math.sqrt(kinematics['seg_car_avg_ax'][seg_num]
                  ** 2 + kinematics['seg_car_avg_ay'][seg_num] ** 2)

    if kinematics['seg_car_avg_a'][seg_num] < 0:
        a = -abs(a)

    car_ax = a * math.sin(kinematics['seg_car_avg_h'][seg_num])
    car_ay = a * math.cos(kinematics['seg_car_avg_h'][seg_num])

    heading = calcu_head(seg_num, kinematics, head)

    ped_vx = v * math.sin(heading)
    ped_vy = v * math.cos(heading)

    car_traj_point = [
        cv_motion_point(tuple(car_loc), (car_vx, car_vy),
                        predict_interval * (i + 1))
        for i in range(num_of_traj_points)
    ]

    ped_traj_point = [
        cv_motion_point(tuple(ped_loc), (ped_vx, ped_vy),
                        predict_interval * (i + 1))
        for i in range(num_of_traj_points)
    ]

    return car_traj_point, ped_traj_point
