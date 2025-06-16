import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, TextBox, Button
from mpl_toolkits.mplot3d import Axes3D
import time
import threading

# ----- 설정 -----
SIMULATION_MODE = True
ARDUINO_PORT = 'COM9'
BAUDRATE = 115200

# ----- 시리얼 연결 -----
arduino = None
if not SIMULATION_MODE:
    try:
        import serial
        arduino = serial.Serial(ARDUINO_PORT, BAUDRATE)
        time.sleep(2)
        print(f"[INFO] Connected to Arduino on {ARDUINO_PORT}")
    except Exception as e:
        print(f"[ERROR] Could not open {ARDUINO_PORT}: {e}")
        exit(1)

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
    if now - last_send_time > 0.05 and arduino:
        try:
            packet = bytearray()
            for theta0, theta1, theta2 in all_leg_angles:
                packet.append(int(np.clip(np.degrees(theta0), 0, 180)))
                packet.append(int(np.clip(np.degrees(theta1), 0, 180)))
                packet.append(int(np.clip(np.degrees(theta2), 0, 180)))
            arduino.write(packet)
            last_send_time = now
        except Exception as e:
            print(f"[WARN] Serial write failed: {e}")

# ----- 시뮬레이터 초기화 -----
fig = plt.figure(figsize=(12, 10))
ax = fig.add_subplot(111, projection='3d')
ax.view_init(elev=30, azim=135)
plt.subplots_adjust(left=0.3, bottom=0.45)

# ----- 다리 라인 및 목표 표시용 -----
leg_lines = []
target_dots = []
for _ in range(6):
    line, = ax.plot([], [], [], 'o-', linewidth=2)
    dot, = ax.plot([], [], [], 'rx')
    leg_lines.append(line)
    target_dots.append(dot)

angle_text = ax.text2D(0.05, 0.92, '', transform=ax.transAxes)


# ----- 슬라이더 UI 구성 -----
sliders = {}
slider_axes = {}
for i in range(6):
    dy = 0.03
    leg_id = i + 1
    for j, axis in enumerate(['x', 'y', 'z']):
        ax_pos = [0.25 + j * 0.18 * 1.3, 0.25 - dy * i, 0.14, 0.02]
        slider_axes[f'{axis}{leg_id}'] = plt.axes(ax_pos)
        sliders[f'{axis}{leg_id}'] = Slider(
            slider_axes[f'{axis}{leg_id}'],
            f'Leg{leg_id} {axis.upper()}',
            -500.0, 500.0,
            valinit=initial_targets[leg_id][j]
        )

# ----- 개별 다리 이동 함수 (역기구학 + 시각화 + 각도 반환) -----
def move_to_leg(leg_num, X, Y, Z):
    base = leg_bases[leg_num]
    offset = np.array([X, Y, Z])
    theta0 = np.arctan2(X, Z)
    planar_dist = np.sqrt(X**2 + Z**2)
    x2d = planar_dist - coxa
    y2d = Y

    cos_theta2 = (x2d**2 + y2d**2 - femur**2 - tibia**2) / (2 * femur * tibia)
    if np.abs(cos_theta2) > 1:
        angles = (np.radians(135), np.radians(135), np.radians(135))
        leg_lines[leg_num-1].set_data([], [])
        leg_lines[leg_num-1].set_3d_properties([])
        target_dots[leg_num-1].set_data([], [])
        target_dots[leg_num-1].set_3d_properties([])
    else:
        theta2 = -np.arccos(cos_theta2)
        k1 = femur + tibia * np.cos(theta2)
        k2 = tibia * np.sin(theta2)
        theta1 = np.arctan2(y2d, x2d) - np.arctan2(k2, k1)
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
        leg_lines[leg_num-1].set_data([base[0], coxa_end[0], femur_end[0], tibia_end[0]],
                                      [base[1], coxa_end[1], femur_end[1], tibia_end[1]])
        leg_lines[leg_num-1].set_3d_properties([base[2], coxa_end[2], femur_end[2], tibia_end[2]])
        target_pos = base + offset
        target_dots[leg_num-1].set_data([target_pos[0]], [target_pos[1]])
        target_dots[leg_num-1].set_3d_properties([target_pos[2]])
        angles = (theta0, theta1, theta2)
    return angles

# ----- 곡선 경로 이동 함수 (1차 베지어 곡선) -----
def lag_1st_Bezier_curve_path_move(leg_num, X1, Y1, Z1, X2, Y2, Z2, X3, Y3, Z3, steps, milliseconds):
    delay = milliseconds / steps / 1000.0
    for t in np.linspace(0, 1, steps):
        x = (1 - t)**2 * X1 + 2 * (1 - t) * t * X3 + t**2 * X2
        y = (1 - t)**2 * Y1 + 2 * (1 - t) * t * Y3 + t**2 * Y2
        z = (1 - t)**2 * Z1 + 2 * (1 - t) * t * Z3 + t**2 * Z2
        sliders[f'x{leg_num}'].set_val(x)
        sliders[f'y{leg_num}'].set_val(y)
        sliders[f'z{leg_num}'].set_val(z)
        update(None)
        time.sleep(delay)

# ----- 직선 경로 이동 함수 -----
def lag_straight_path_move(leg_num, X1, Y1, Z1, X2, Y2, Z2, steps, milliseconds):
    delay = milliseconds / steps / 1000.0
    for t in np.linspace(0, 1, steps):
        x = (1 - t) * X1 + t * X2
        y = (1 - t) * Y1 + t * Y2
        z = (1 - t) * Z1 + t * Z2
        sliders[f'x{leg_num}'].set_val(x)
        sliders[f'y{leg_num}'].set_val(y)
        sliders[f'z{leg_num}'].set_val(z)
        update(None)
        time.sleep(delay)

# ----- 업데이트 함수 -----
def update(val):
    angles_display = []
    all_leg_angles = []
    for i in range(1, 7):
        X = sliders[f'x{i}'].val
        Y = sliders[f'y{i}'].val
        Z = sliders[f'z{i}'].val
        angles = move_to_leg(i, X, Y, Z)
        all_leg_angles.append(angles)
        angles_display.append(f"Leg {i}: θ0={np.degrees(angles[0]):.1f}°, θ1={np.degrees(angles[1]):.1f}°, θ2={np.degrees(angles[2]):.1f}°")

    send_angles(all_leg_angles)
    angle_text.set_text('\n'.join(angles_display))
    ax.set_xlim(-500, 500)
    ax.set_ylim(500, -500)
    ax.set_zlim(-500, 500)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    fig.canvas.draw_idle()

for s in sliders.values():
    s.on_changed(update)

update(None)

# ----- 버튼을 통한 테스트 실행 -----
ax_test = plt.axes([0.05, 0.01, 0.15, 0.04])
btn_test = Button(ax_test, 'Test Leg Move')

def test_lag_move(event):
    def sequence():
        lag_straight_path_move(1, -147, -100, 255, -147, -100, -255, 100, 3000)
        time.sleep(1)
        lag_1st_Bezier_curve_path_move(1, -147, -100, -255, -147, -100, 255, -147, 100, 0, 100, 3000)
        time.sleep(1)
        lag_straight_path_move(1, -147, -100, 255, -147, -100, -255, 100, 3000)
        time.sleep(1)
        lag_1st_Bezier_curve_path_move(1, -147, -100, -255, -147, -100, 255, -147, 100, 0, 100, 3000)
    thread = threading.Thread(target=sequence)
    thread.start()

btn_test.on_clicked(test_lag_move)

plt.show()
