import matplotlib.pyplot as plt
import matplotlib
import pandas as pd
import numpy as np
import fire
from pathlib import Path
matplotlib.use('TkAgg')

def plot_result(path: Path):
    df_opt = pd.read_csv(path)
    plt.figure(figsize=(10, 5))
    plt.subplot(1, 2, 1)
    plt.plot(df_opt['x_left'], df_opt['y_left'], label="Left Boundary", color='blue', linestyle='--')
    plt.plot(df_opt['x_right'], df_opt['y_right'], label="Right Boundary", color='red', linestyle='--')
    plt.plot(df_opt['x_center'], df_opt['y_center'], label="Center Points", color='black', marker='o', linestyle='-', markersize=4)
    plt.plot(df_opt['x_opt'], df_opt['y_opt'], label="Optimized Points", color='green', marker='o', linestyle='-', markersize=4)
    plt.title("Left Boundary, Right Boundary, Center Points and Optimized Points")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.legend()
    plt.grid(True)

    plt.subplot(1, 2, 2)
    u_values = np.linspace(0, 1, len(df_opt))
    plt.plot(u_values, df_opt['k_center'], 'k-', label='Center spline urvature')
    plt.plot(u_values, df_opt['k_opt'], 'g-', label='Optimized Spline Curvature')
    plt.title('Curvature along the Spline')
    plt.xlabel('u (parameter)')
    plt.ylabel('Curvature')
    plt.legend()
    plt.grid()

    plt.show()  

if __name__ == '__main__':
    fire.Fire(plot_result)
