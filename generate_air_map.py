import numpy as np

import matplotlib.pyplot as plt

import pandas as pd

class TwoNomal():
    def __init__(self,mu1,mu2,sigma1,sigma2):
        self.mu1 = mu1
        self.sigma1 = sigma1
        self.mu2 = mu2
        self.sigma2 = sigma2

    def singledensity(self,x):
        mu1 = self.mu1
        sigma1 = self.sigma1
        
        N1 = np.sqrt(2 * np.pi * np.power(sigma1, 2))
        fac1 = np.power(x - mu1, 2) / np.power(sigma1, 2)
        density1 = np.exp(-fac1/2)/N1
        density = density1 * 10000
        return density

    def doubledensity(self,x):
            mu1 = self.mu1
            sigma1 = self.sigma1
            mu2 = self.mu2
            sigma2 = self.sigma1
            N1 = np.sqrt(2 * np.pi * np.power(sigma1, 2))
            fac1 = np.power(x - mu1, 2) / np.power(sigma1, 2)
            density1=np.exp(-fac1/2)/N1

            N2 = np.sqrt(2 * np.pi * np.power(sigma2, 2))
            fac2 = np.power(x - mu2, 2) / np.power(sigma2, 2)
            density2=np.exp(-fac2/2)/N2
            #print(density1,density2)
            density=(0.5*density2+0.5*density1) * 10000
            return density

def tow_dimensional_nomal(x_mesh, y_mesh, mu_1, mu_2, sigma, multiple):
    z = 1/(2 * np.pi * (sigma**2))
    z = z * np.exp(-((x_mesh - mu_1)**2+(y_mesh - mu_2)**2)/(2 * sigma**2))
    return z * multiple

def main():
    N2 = TwoNomal(100,300,50,50)

    #创建等差数列作为X
    X = np.arange(0,400,4)
    Y = np.arange(0,400,4)
    #print(X)
    Xv = N2.doubledensity(X)
    Yv = N2.doubledensity(Y)

    Z_pred = [[0 for i in range(len(Xv))] for i in range(len(Yv))]
    for i in range(len(Xv)):
        for j in range(len(Yv)):
            Z_pred[i][j] = Xv[i] + Yv[j]

    # print(Z_pred.shape)

    X_mesh,Y_mesh = np.meshgrid(X, Y)
    plt.pcolormesh(X_mesh, Y_mesh, Z_pred, shading='gouraud', cmap='jet')

    #print(Y)
    # plt.plot(X,Y,'b-',linewidth=3)

    df = pd.DataFrame(Z_pred)
    df.to_csv('data_set/air_map_simulation_nxn.csv', index=False)

    plt.show()

if __name__=='__main__':
    main()