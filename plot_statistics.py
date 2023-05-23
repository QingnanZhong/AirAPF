import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def plot_statistics(trials):
    num_rows = []
    max_tiaqi = []
    for i in range(trials):
        df = pd.read_csv('DataSet/tiaqi/tiaqi_%d.csv'%(i+1))
        max_tiaqi.append(df['max'])
        num_rows.append(df.shape[0])

    for i in range(trials):
        xx = np.arange(0, num_rows[i], 1)
        plt.plot(xx, max_tiaqi[i])

    plt.xlabel("Time(sec)")
    plt.ylabel("Max(TIAQI)")
    plt.savefig('Figs/0522/statistics_max_tiaqi_%d.png'%trials, bbox_inches='tight', dpi=500, pad_inches=0.0)
    plt.show()

def plot_pollution_num(trials):
    num_rows = []
    p_num = []
    for i in range(trials):
        df = pd.read_csv('DataSet/p_sum/p_sum_%d.csv'%(i+1))
        p_num.append(np.array([df['hlp_sum'], df['mlp_sum'], df['llp_sum']]))
        num_rows.append(df.shape[0])
    
    for i in range(trials):
        xx = np.arange(0, num_rows[i], 1)
        if i==0:
            plt.plot(xx, p_num[i][0], "-r", label="heavy pollution")
            plt.plot(xx, p_num[i][1], "-g", label="medium pollution")
            plt.plot(xx, p_num[i][2], "-b", label="low pollution")
        else:
            plt.plot(xx, p_num[i][0], "-r")
            plt.plot(xx, p_num[i][1], "-g")
            plt.plot(xx, p_num[i][2], "-b")

    plt.xlabel("Time(sec)")
    plt.ylabel("Sum(TIAQI)")
    plt.legend()
    plt.savefig('Figs/0522/statistics_pollution_sum_%d.png'%trials, bbox_inches='tight', dpi=500, pad_inches=0.0)
    plt.show()


if __name__=='__main__':
    plot_statistics(10)