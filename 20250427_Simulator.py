import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from mpl_toolkits.mplot3d import Axes3D
import serial
import time

# ----- 아두이노 연결 -----
arduino = serial.Serial('COM9', 115200)
time.sleep(2)  # 연결 안정화 대기

# ----- 링크 길이 -----
coxa = 70
femur = 120
tibia = 193

# ----- 다리 기본 목표 위치 -----
initial_targets = {
    1: np.array([-147.264, -100.000, 255.069]),
    2: np.array([147.264, -100.000, 255.069]),
    3: np.array([-294.528, -100.000, 0.000]),
    4: np.array([294.528, -100.000, 0.000]),
    5: np.array([-147.264, -100.000, -255.069]),
    6: np.array([147.264, -100.000, -255.069]),
}

# ----- 각 다리 베이스 위치 -----
leg_bases = {
    1: np.array([-113.280, 0.000, 196.210]),
    2: np.array([113.280, 0.000, 196.210]),
    3: np.array([-226.560, 0.000, 0.000]),
    4: np.array([226.560, 0.000, 0.000]),
    5: np.array([-113.280, 0.000, -196.210]),
    6: np.array([113.280, 0.000, -196.210]),
}

# ----- 아두이노로 각도 전송 함수 -----
last_send_time = 0

def send_angles(all_leg_angles):
    global last_send_time
    now = time.time()
    if now - last_send_time > 0.05:  # 50ms 간격 제한
        packet = bytearray()
        for theta0, theta1, theta2 in all_leg_angles:
            packet.append(int(np.clip(np.degrees(theta0), 0, 180)))
            packet.append(int(np.clip(np.degrees(theta1), 0, 180)))
            packet.append(int(np.clip(np.degrees(theta2), 0, 180)))
        try:
            arduino.write(packet)
            last_send_time = now
        except serial.SerialException:
            print("Warning: Serial write failed, check connection.")

# ----- 3D 시뮬레이터 설정 -----
fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111, projection='3d')
ax.view_init(elev=30, azim=135)
plt.subplots_adjust(left=0.25, bottom=0.35)

# 슬라이더 만들기
sliders = {}
slider_axes = {}

for i in range(6):
    dy = 0.03
    leg_id = i + 1
    for j, axis in enumerate(['x', 'y', 'z']):
        ax_pos = [0.25 + j * 0.25, 0.25 - dy * i, 0.20, 0.02]
        slider_axes[f'{axis}{leg_id}'] = plt.axes(ax_pos)
        sliders[f'{axis}{leg_id}'] = Slider(
            slider_axes[f'{axis}{leg_id}'],
            f'Leg{leg_id} {axis.upper()}',
            -500.0,
            500.0,
            valinit=initial_targets[leg_id][j]
        )

# 다리 라인, 목표 점 표시용
leg_lines = []
target_dots = []

for _ in range(6):
    line, = ax.plot([], [], [], 'o-', linewidth=2)
    dot, = ax.plot([], [], [], 'rx')
    leg_lines.append(line)
    target_dots.append(dot)

angle_text = ax.text2D(0.05, 0.92, '', transform=ax.transAxes)

# ----- 업데이트 함수 -----
def update(val):
    angles_display = []
    all_leg_angles = []  # 아두이노에 보낼 각도 리스트

    for i, base in leg_bases.items():
        offset = np.array([
            sliders[f'x{i}'].val,
            sliders[f'y{i}'].val,
            sliders[f'z{i}'].val,
        ])
        X, Y, Z = offset

        theta0 = np.arctan2(X, Z)
        planar_dist = np.sqrt(X**2 + Z**2)
        x2d = planar_dist - coxa
        y2d = Y

        d = np.sqrt(x2d**2 + y2d**2)
        cos_theta2 = (x2d**2 + y2d**2 - femur**2 - tibia**2) / (2 * femur * tibia)

        if np.abs(cos_theta2) <= 1:
            theta2 = -np.arccos(cos_theta2)
            k1 = femur + tibia * np.cos(theta2)
            k2 = tibia * np.sin(theta2)
            theta1 = np.arctan2(y2d, x2d) - np.arctan2(k2, k1)

            # 다리 좌표 계산
            coxa_end = base + np.array([coxa * np.sin(theta0), 0, coxa * np.cos(theta0)])
            femur_end = coxa_end + np.array([
                femur * np.cos(theta1) * np.sin(theta0),
                femur * np.sin(theta1),
                femur * np.cos(theta1) * np.cos(theta0)
            ])
            tibia_end = femur_end + np.array([
                tibia * np.cos(theta1 + theta2) * np.sin(theta0),
                tibia * np.sin(theta1 + theta2),
                tibia * np.cos(theta1 + theta2) * np.cos(theta0)
            ])

            leg_lines[i-1].set_data([base[0], coxa_end[0], femur_end[0], tibia_end[0]],
                                    [base[1], coxa_end[1], femur_end[1], tibia_end[1]])
            leg_lines[i-1].set_3d_properties([base[2], coxa_end[2], femur_end[2], tibia_end[2]])

            target_pos = base + offset
            target_dots[i-1].set_data([target_pos[0]], [target_pos[1]])
            target_dots[i-1].set_3d_properties([target_pos[2]])

            angles_display.append(f"Leg {i}: θ₀={np.degrees(theta0):.1f}°, θ₁={np.degrees(theta1):.1f}°, θ₂={np.degrees(theta2):.1f}°")

            # 리스트에 각도 저장
            all_leg_angles.append((theta0, theta1, theta2))
        else:
            # 목표 범위 벗어나면 빈 값
            leg_lines[i-1].set_data([], [])
            leg_lines[i-1].set_3d_properties([])
            target_dots[i-1].set_data([], [])
            target_dots[i-1].set_3d_properties([])
            angles_display.append(f"Leg {i}: Out of reach")

            # 안전하게 90도 중립값 보내기
            all_leg_angles.append((np.radians(90), np.radians(90), np.radians(90)))

    # 아두이노로 모든 다리 각도 전송
    send_angles(all_leg_angles)

    # 디스플레이 업데이트
    angle_text.set_text('\n'.join(angles_display))
    ax.set_xlim(-500, 500)
    ax.set_ylim(-500, 500)
    ax.set_zlim(-500, 500)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    fig.canvas.draw_idle()

# 슬라이더와 업데이트 연결
for s in sliders.values():
    s.on_changed(update)

# 초기 상태 업데이트
update(None)

# 3D 시뮬레이터 실행
plt.show()
