import matplotlib.pyplot as plt
import numpy
import math
import numpy as np
import random

# Vehicle parameters
LENGTH = 2.0  # [m]
WIDTH = 2.0  # [m]
BACKTOWHEEL = 1.0  # [m]
WHEEL_LEN = 0.3  # [m]
WHEEL_WIDTH = 0.2  # [m]
TREAD = 0.7  # [m]
WB = 2.5  # [m]
DISINFECTION_RADIUS = 2 # [m]

def plot_path(path, color='y', linestyle=None, label=None):
    start_x, start_y = path[0]
    goal_x, goal_y = path[-1]

    # plot path
    path_arr = numpy.array(path)
    plt.plot(path_arr[:, 0], path_arr[:, 1], c=color, linestyle=linestyle, label=label)
    # plot start point
    plt.plot(start_x, start_y, 'r*')
    # plot goal point
    plt.plot(goal_x, goal_y, 'g*')

def plot_obstacle(obstacle_node):
    oy = obstacle_node[:,0]
    ox = obstacle_node[:,1]
    plt.scatter(ox, oy, s=500, c='gray')

def plot_boundary(boundary_coordinates):
    plt.Rectangle((1,1), 3, 25)

def list_sampling(ls, divisor):
    UUx = np.zeros((round(len(ls)/divisor), round(len(ls[0])/divisor)))
    for ix in range(round(len(ls)/divisor)):
        for iy in range(round(len(ls[0])/divisor)):
            UUx[ix][iy] = Ux[ix*2][iy*divisor]

    return UUx

def limit_quiver_length(ux, uy, limit_scale):
    for i in range(len(ux)):
        for j in range(len(ux[0])):
            if abs(ux[i][j]) >= limit_scale or abs(uy[i][j]) >= limit_scale:
                ux[i][j] = ux[i][j]/2
                uy[i][j] = uy[i][j]/2
    return ux, uy

def plot_map_and_quiver(x_mesh, y_mesh, pmap):
    plt.pcolormesh(x_mesh, y_mesh, pmap, shading='gouraud', cmap='jet', vmin=0, vmax=200)
    # plt.colorbar()
    Uy, Ux = np.gradient(pmap)
    # 计算向量长度
    mag = np.sqrt(Ux**2 + Uy**2)
    # 对向量长度进行规范化
    u_norm, v_norm = Ux/mag, Uy/mag
    # 设置缩放因子，控制箭头长度的大小
    scale = 0.3 / np.max(mag)
    u_norm, v_norm = u_norm * scale, v_norm * scale
    
    plt.quiver(x_mesh, y_mesh, u_norm, v_norm)
    

def plot_car(position, yaw, steer=0.0, cabcolor="-r", truckcolor="-k"):  # pragma: no cover

    x ,y = position

    outline = np.array([[-BACKTOWHEEL, (LENGTH - BACKTOWHEEL), (LENGTH - BACKTOWHEEL), -BACKTOWHEEL, -BACKTOWHEEL],
                        [WIDTH / 2, WIDTH / 2, - WIDTH / 2, -WIDTH / 2, WIDTH / 2]])

    fr_wheel = np.array([[WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
                         [-WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD]])

    rr_wheel = np.copy(fr_wheel)

    fl_wheel = np.copy(fr_wheel)
    fl_wheel[1, :] *= -1
    rl_wheel = np.copy(rr_wheel)
    rl_wheel[1, :] *= -1

    Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                     [-math.sin(yaw), math.cos(yaw)]])
    Rot2 = np.array([[math.cos(steer), math.sin(steer)],
                     [-math.sin(steer), math.cos(steer)]])

    fr_wheel = (fr_wheel.T.dot(Rot2)).T
    fl_wheel = (fl_wheel.T.dot(Rot2)).T
    fr_wheel[0, :] += WB
    fl_wheel[0, :] += WB

    fr_wheel = (fr_wheel.T.dot(Rot1)).T
    fl_wheel = (fl_wheel.T.dot(Rot1)).T

    outline = (outline.T.dot(Rot1)).T
    rr_wheel = (rr_wheel.T.dot(Rot1)).T
    rl_wheel = (rl_wheel.T.dot(Rot1)).T

    outline[0, :] += x
    outline[1, :] += y
    fr_wheel[0, :] += x
    fr_wheel[1, :] += y
    rr_wheel[0, :] += x
    rr_wheel[1, :] += y
    fl_wheel[0, :] += x
    fl_wheel[1, :] += y
    rl_wheel[0, :] += x
    rl_wheel[1, :] += y

    plt.plot(np.array(outline[0, :]).flatten(),
             np.array(outline[1, :]).flatten(), truckcolor)
    # plt.plot(np.array(fr_wheel[0, :]).flatten(),
    #          np.array(fr_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(rr_wheel[0, :]).flatten(),
             np.array(rr_wheel[1, :]).flatten(), truckcolor)
    # plt.plot(np.array(fl_wheel[0, :]).flatten(),
    #          np.array(fl_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(rl_wheel[0, :]).flatten(),
             np.array(rl_wheel[1, :]).flatten(), truckcolor)
    plt.plot(x, y, "*")
    
    cir = plt.Circle(xy = (x, y), radius=DISINFECTION_RADIUS, alpha=0.5)
    plt.gca().add_patch(cir)

    return DISINFECTION_RADIUS

def get_motion_model():
    # dx, dy
    motion = [[1, 0],
                [0, 1],
                [-1, 0],
                [0, -1],
                [-1, -1],
                [-1, 1],
                [1, -1],
                [1, 1]]
    return motion

def find_list_max_value(list):
    max_ap = 0
    for ix in range(len(list)):
        for iy in range(len(list[0])):
            if max_ap < list[ix][iy]:
                max_ap = list[ix][iy]
                gx, gy = ix, iy
            # if list[ix][iy] < 0:
            #     print(list[ix][iy])
    return max_ap, gx, gy

def find_list_max_value_2(list):
    max_ap = 0
    for ix in range(len(list)):
        for iy in range(len(list[0])):
            if max_ap < list[ix][iy]:
                max_ap = list[ix][iy]
                gx, gy = ix, iy
            # if list[ix][iy] < 0:
            #     print(list[ix][iy]) 
    return max_ap, gx, gy

def generate_boundary_coordinates(pmap):
    # 定义边界坐标
    boundary_coordinates = []
    map_width = len(pmap)
    map_height = len(pmap[0])

    # 生成上边界
    for i in range(map_width):
        boundary_coordinates.append((i, 0))

    # 生成右边界
    for i in range(1, map_height):
        boundary_coordinates.append((map_width - 1, i))

    # 生成下边界
    for i in range(map_width - 2, -1, -1):
        boundary_coordinates.append((i, map_height - 1))

    # 生成左边界
    for i in range(map_height - 2, 0, -1):
        boundary_coordinates.append((0, i))

    # 打印边界坐标
    # print(boundary_coordinates)

    return boundary_coordinates


def random_points(x_min, x_max, y_min, y_max, num):
    points = np.array([[random.randint(x_min, x_max), random.randint(y_min, y_max)] for i in range(num)])
    return points