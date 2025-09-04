import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

# 데이터 로드
df = pd.read_csv("/home/siwon/study_ws/trajectory.csv", header=0, names=["x", "y", "vx", "vy"])

# Figure 및 Subplot 구성
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 5))
ax1.set_aspect('equal')
ax2.set_aspect('equal')

# --- 왼쪽: 위치 궤적만 ---
ax1.set_xlim(df["x"].min() - 1, df["x"].max() + 1)
ax1.set_ylim(df["y"].min() - 1, df["y"].max() + 1)
ax1.set_xlabel("X")
ax1.set_ylabel("Y")
ax1.set_title("Trajectory (Position Only)")
ax1.grid()

point, = ax1.plot([], [], 'bo', markersize=8, label="Position")
trail, = ax1.plot([], [], 'r--', linewidth=1, label="Path")

# --- 오른쪽: vx, vy, v_total ---
ax2.set_xlim(-1.5, 1.5)
ax2.set_ylim(-1.5, 1.5)
ax2.set_xlabel("vx")
ax2.set_ylabel("vy")
ax2.set_title("Velocity Components")
ax2.grid()

vec_vx = ax2.quiver(0, 0, 0, 0, angles='xy', scale_units='xy', scale=1, color='g', label='vx')
vec_vy = ax2.quiver(0, 0, 0, 0, angles='xy', scale_units='xy', scale=1, color='m', label='vy')
vec_vtotal = ax2.quiver(0, 0, 0, 0, angles='xy', scale_units='xy', scale=1, color='b', label='v_total')

def init():
    point.set_data([], [])
    trail.set_data([], [])
    vec_vx.set_UVC(0, 0)
    vec_vy.set_UVC(0, 0)
    vec_vtotal.set_UVC(0, 0)
    return point, trail, vec_vx, vec_vy, vec_vtotal

def update(frame):
    x = df["x"].iloc[frame]
    y = df["y"].iloc[frame]
    vx = df["vx"].iloc[frame]
    vy = df["vy"].iloc[frame]

    # 왼쪽: 위치만 표시
    point.set_data(x, y)
    trail.set_data(df["x"].iloc[:frame+1], df["y"].iloc[:frame+1])

    # 오른쪽: vx, vy, v_total 화살표
    vec_vx.set_UVC([vx], [0])
    vec_vy.set_UVC([0], [vy])
    vec_vtotal.set_UVC([vx], [vy])

    return point, trail, vec_vx, vec_vy, vec_vtotal

ani = animation.FuncAnimation(
    fig, update, frames=len(df), init_func=init, interval=80, blit=False)

ax1.legend(loc='upper right', fontsize=10)
ax2.legend(loc='upper right', fontsize=10)

plt.tight_layout()
plt.show()
