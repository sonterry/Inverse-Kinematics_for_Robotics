import serial
import time

# 아두이노와 연결 (COM 포트는 환경에 맞게 설정)
arduino = serial.Serial('COM9', 9600)
time.sleep(2)  # 연결 안정화 대기

def move_servo(angle):
    arduino.write(f"{angle}\n".encode())  # 아두이노로 각도 전송
    time.sleep(0.02)  # 서보가 이동할 시간

# 0 -> 180도
for angle in range(0, 181):
    move_servo(angle)

# 180 -> 0도
for angle in range(180, -1, -1):
    move_servo(angle)

