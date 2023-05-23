import pandas as pd
import numpy as np

class GV:
    df = pd.read_csv('DataSet/air_map_simulation.csv')
    pmap = df.values.tolist()
    robot_radius = 3.0  # robot radius [m]
    obstacle_node = np.array([[5, 10], [13, 5], [13, 18], [18, 15]]) # obstacle position list [m]
    start_node = [10, 10]
    show_animation = False
    avg_tiaqi = []
    max_tiaqi = []
    ax_tiaqi = []
    high_level_pollution = []
    hlp_sum = []
    mlp_sum = []
    llp_sum = []

