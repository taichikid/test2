from pymavlink import mavutil
import math

def get_yaw_degrees(master):
    # ドローンの方位を取得
    msg = master.recv_match(type='ATTITUDE', blocking=True)
    roll = msg.roll
    pitch = msg.pitch
    yaw = msg.yaw

    # 方位を度に変換
    yaw_degrees = math.degrees(yaw)

    return yaw_degrees

def calculate_heading_error(yaw_degrees):
    # 北を基準としたずれを計算
    heading_error = 360 - yaw_degrees
    if heading_error >= 360:
        heading_error -= 360

    return heading_error

def main():
    # Pixhawkに接続
    master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

    try:
        while True:
            # ドローンの方位を取得
            yaw_degrees = get_yaw_degrees(master)

            # 北からのずれを計算
            heading_error = calculate_heading_error(yaw_degrees)

            # 結果を出力
            print(f'Yaw Degrees: {yaw_degrees:.2f}, Heading Error: {heading_error:.2f} degrees')

    except KeyboardInterrupt:
        print('終了します。')

if __name__ == "__main__":
    main()
