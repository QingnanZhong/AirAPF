import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import math
import copy
import time
from global_var import GV
from Utils.util import *
from calc_potential_field import *
from generate_air_map import *
from matplotlib.animation import FFMpegWriter


GRID_SIZE = 0.5
BOUNDARY_LIMIT = 2 # [m]

class PathManager:
    def __init__(self, pmap, start_node, obstacle_node, robot_radius):
        self.pmap = pmap
        self.start_node = random_points(robot_radius, len(pmap)-robot_radius,\
                                        robot_radius, len(pmap[0])-robot_radius, 1)
        # print('start_node = ', self.start_node)
        # self.start_node = start_n
        # self.obstacle_node = obstacle_node
        self.obstacle_node = random_points(robot_radius, len(pmap)-robot_radius,\
                                        robot_radius, len(pmap[0])-robot_radius, 4)
        # print('obstacle node = ', self.obstacle_node)

        self.robot_radius = robot_radius
        x_t = np.linspace(0, len(pmap)-1, len(pmap))
        y_t = np.linspace(0, len(pmap[0])-1, len(pmap[0]))
        # print('x_t, y_t =', x_t, y_t)
        self.X_mesh, self.Y_mesh = np.meshgrid(x_t, y_t)
        plt.plot(self.start_node[0][0], self.start_node[0][1] ,color='m',marker='*')
        self.path = None
        self.car_x, self.car_y = self.start_node[0]
        self.path = ([[self.car_x, self.car_y]])
        self.yaw = ([math.atan2(self.car_y, self.car_x)])
        self.boundary_coordinates = generate_boundary_coordinates(self.pmap)
        self.obstacle = np.concatenate((self.obstacle_node, self.boundary_coordinates), axis=0)

    def find_current_TIAQI_max(self):
        pmap_copy = copy.deepcopy(self.pmap)
        for x,y in self.obstacle:
            pmap_copy[x][y] = 0

        max_v, _, _ = find_list_max_value_2(pmap_copy)
        flatten_lst = [y for x in pmap_copy for y in x]
        sum_pmap = 0
        for p in flatten_lst:
            if p > 0:
                sum_pmap += p
        avg = sum_pmap / (len(flatten_lst) - len(self.obstacle))

        hlp = 0
        for p in flatten_lst:
            if p > 150:
                hlp += p*4
            elif p > 100:
                hlp += p*2   

        hlp_sum, mlp_sum, llp_sum = 0, 0, 0
        hs, ms, ls = 1, 1, 1
        for p in flatten_lst:
            if p > 150:
                hlp_sum += p
                hs += 1
            elif p > 100:
                mlp_sum += p
                ms += 1
            elif p > 50:
                llp_sum += p    
                ls += 1

        # print('ps, ms, ls =', hs, ms, ls)
        # hlp_sum = hlp_sum / hs
        # mlp_sum = mlp_sum / ms
        # llp_sum = llp_sum / ls

        return max_v, avg, hlp, hlp_sum, mlp_sum, llp_sum

    # position: is position of car
    # value: is the map that will be whittled #
    def circle_whittle_the_value(self, position, value):
        x, y = position
        X = np.arange(0, len(value), 1)
        Y = np.arange(0, len(value[0]), 1)
        x_mesh, y_mesh = np.meshgrid(X, Y)
        z = tow_dimensional_nomal(x_mesh, y_mesh, x, y, 4, 300)
        for ix in range(len(value)):
            for iy in range(len(value[0])):
                    value[ix][iy] -= z[ix,iy] 
        return z

    def calc_potential_field(self):
        # calc each potential
        for ix in range(len(self.pmap)):
            for iy in range(len(self.pmap[0])):
                ug = 0
                uo = calc_repulsive_potential(ix, iy, list(self.obstacle[:,0]), list(self.obstacle[:,1]), self.robot_radius)
                uf = ug + uo
                if uf < 0:
                    print('uf = ', uf)
                self.pmap[ix][iy] -= float(uf)
        return self.pmap

    def find_path_first_global(self, start_node):
        # find the max value which is air pollutant
        max_ap, gx, gy = find_list_max_value(self.pmap)

        # search path
        d = np.hypot(start_node[0] - gx, start_node[1] - gy)
        ix = round(start_node[0])
        iy = round(start_node[1])
        # print(ix, iy)
        motion = get_motion_model()
        while d >= GRID_SIZE:
            minp = 0
            minix, miniy = -1, -1

            for i, _ in enumerate(motion):
                inx = int(ix + motion[i][0])
                iny = int(iy + motion[i][1])
                if inx >= len(self.pmap) or iny >= len(self.pmap[0]):
                    p = -float("inf")  # outside area
                else:
                    p = self.pmap[iny][inx]
                   
                if minp < p:
                    minp = p
                    minix = inx
                    miniy = iny

            # Exit if entered the local minimum point 
            if minix == ix and miniy == iy:
                break

            ix = minix
            iy = miniy

            d = np.hypot(gx - ix, gy - iy)
            # print(d)

            self.path.append([ix, iy])
            self.yaw.append(math.atan2(iy, ix))

        print("Find Goal!!")
        
        return self.path

    def find_path_auto_global(self, start_node):
        # find the max value which is air pollutant
        max_ap, gx, gy = find_list_max_value(self.pmap)

        # search path
        d = np.hypot(self.path[-1][0] - gx, self.path[-1][1] - gy)
        ix = round(self.path[-1][0])
        iy = round(self.path[-1][1])
        # print(ix, iy, self.pmap[ix][iy])
        motion = get_motion_model()
        minp = -float("inf")
        minix, miniy = -1, -1

        for i, _ in enumerate(motion):
            inx = int(ix + motion[i][0])
            iny = int(iy + motion[i][1])
            if inx >= len(self.pmap) or iny >= len(self.pmap[0]):
                p = -float("inf")  # outside area
            else:
                p = self.pmap[iny][inx]
                # print('inx, iny, p, self.pmap[iny][inx] = ',inx, iny, p, self.pmap[iny][inx])

            if minp < p:
                minp = p
                minix = inx
                miniy = iny

        if minix != -1 and miniy != -1:
            d = np.hypot(gx - ix, gy - iy)
            # print('minix, miniy, minp = ', minix, miniy, minp)
            self.path.append([minix, miniy])
            dx = minix - self.path[-2][0]
            dy = miniy - self.path[-2][1] 
            self.yaw.append(math.atan2(dy, dx))
            # print("Update Goal!!")
    
        return max_ap

    def plot_whittle_map(self):
        self.find_path_auto_global(self.path[0]) 
        z = self.circle_whittle_the_value(self.path[0], self.pmap)
        plt.pcolormesh(self.X_mesh, self.Y_mesh, z)
        plt.colorbar()
        plt.tight_layout()
        plt.axis("square")
        plt.savefig('Figs/0515_02/whittle_map.png', bbox_inches='tight', dpi=500, pad_inches=0.0)
        plt.show()
        
    def add_to_csv(self):
        df = pd.DataFrame()
        df['hlp_sum'] = GV.hlp_sum
        df['mlp_sum'] = GV.mlp_sum
        df['llp_sum'] = GV.llp_sum
        df.to_csv('DataSet/p_sum/p_sum_10.csv', index=False)
        df = pd.DataFrame()
        df['avg'] = GV.avg_tiaqi
        df['max'] = GV.max_tiaqi
        df.to_csv('DataSet/tiaqi/tiaqi_10.csv', index=False)

    def plot_all(self):
        # if open the function, the path is ahead of current potential field
        # self.find_path_first_global(self.path[-1])
        # plt.ion()
        # plot_path(self.path, color='cyan', label='origin path')
        # plot_map_and_quiver(self.X_mesh, self.Y_mesh, self.pmap)
        fig = plt.gcf()
        plt.rcParams['animation.ffmpeg_path'] = 'D:\\Tool\\ffmpeg-6.0-full_build\\bin\\ffmpeg.exe'
        metadata = dict(title='test', artist='Matplotlib',comment='test')
        writer = FFMpegWriter(fps=10, metadata=metadata)
        date = time.strftime("%m-%d_%H-%M", time.localtime()) 
        with writer.saving(fig, 'Results/' + 'AirAPF_%s.mp4'%date, 300):
            i = 0
            while True:
                plt.clf()
                plot_path(self.path, color='cyan', label='origin path')
                plot_map_and_quiver(self.X_mesh, self.Y_mesh, self.pmap)
                plot_obstacle(self.obstacle_node)
                plot_car(self.path[i], self.yaw[i])
                self.circle_whittle_the_value(self.path[i], self.pmap)
                self.find_path_auto_global(self.path[i]) 

                maxTIAQI, avg, hlp, hlp_sum, mlp_sum, llp_sum = self.find_current_TIAQI_max()
                GV.avg_tiaqi.append(avg)
                GV.max_tiaqi.append(maxTIAQI)
                GV.hlp_sum.append(hlp_sum)
                GV.mlp_sum.append(mlp_sum)
                GV.llp_sum.append(llp_sum)

                print('maxTIAQI, avg = ', maxTIAQI, avg)
                if maxTIAQI < 50:
                    break

                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',
                        lambda event: [exit(0) if event.key == 'escape' else None])

                plt.pause(0.0001)
                plt.axis("square")
                plt.tight_layout()

                if i % 10 == 0 and GV.show_animation:
                    plt.savefig('Figs/0515_03/APF_step_%d.png'%i, bbox_inches='tight', dpi=500, pad_inches=0.0)

                i += 1
                if i >= len(self.path):
                    i = len(self.path)-1

                if GV.show_animation:
                    writer.grab_frame()

        self.add_to_csv()
        xx = np.arange(0, i+1, 1)
        plt.subplots()
        plt.plot(xx, GV.avg_tiaqi, "-r", label="Average TIAQI")
        plt.plot(xx, GV.max_tiaqi, "-b", label="Max TIAQI")
        plt.xlabel("Time(sec)")
        plt.ylabel("TIAQI")
        plt.legend()
        plt.subplots()
        plt.plot(xx, GV.hlp_sum, "-r", label="heavy pollution")
        plt.plot(xx, GV.mlp_sum, "-g", label="medium pollution")
        plt.plot(xx, GV.llp_sum, color='blue', label="low pollution")
        plt.xlabel("Time(sec)")
        plt.ylabel("Sum(TIAQI)")
        plt.legend()
        plt.savefig('Figs/0517_01/Avg_TIAQI_%s.png'%date, bbox_inches='tight', dpi=500, pad_inches=0.0)
        plt.show()
    
def main():
    print("potential_field_planning start")
    pM = PathManager(GV.pmap, GV.start_node, GV.obstacle_node, GV.robot_radius)
    # only calc repulsive potential, because we use air map as attractive potential 
    pM.calc_potential_field()
    pM.plot_all()

if __name__ == '__main__':
    print(__file__ + " start!!")
    main()
    print(__file__ + " Done!!")