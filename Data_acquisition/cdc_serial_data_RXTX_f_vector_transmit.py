import serial
import struct
import threading
import queue
import time
import os
import sys

###########################################################################
# ============================ USER SETTINGS =============================
###########################################################################

SERIAL_PORT   = 'COM3'    # CDC 포트 번호 (필요 시 수정)
BAUD_RATE     = 921600
TIMEOUT_SEC   = 1.0

SOF_VALUE     = 0xAA55    # little-endian: 0x55 0xAA
QUEUE_MAXSIZE = 5000

# CSV 저장 관련
FLUSH_EVERY   = 500       # 몇 라인마다 디스크에 flush 할지

# 전역 객체
ser = None
data_queue = queue.Queue(maxsize=QUEUE_MAXSIZE)
is_running = True

###########################################################################
# ======================= ▼ F-VECTOR CONFIG AREA ▼ ========================
###########################################################################

TRAJ_ID_LFLEX = 1.0  # Left Flexion
TRAJ_ID_LEXT  = 2.0  # Left Extension
TRAJ_ID_RFLEX = 3.0  # Right Flexion
TRAJ_ID_REXT  = 4.0  # Right Extension
TRAJ_ID_NONE  = 5.0  # None

def fvec(m, tmax, delay, tend):
    return [float(m), float(tmax), float(delay), float(tend)]

F_END = [-1.0, 0.0, 0.0, 0.0]

LFlex_list = [
    TRAJ_ID_LFLEX,
    *fvec(0,  2.5,  0.0, 500.0),
    *F_END,
]

LExt_list = [
    TRAJ_ID_LEXT,
    *fvec(0,  -1.0,  0.0, 180.0),
    *fvec(1,  -0.5, 20.0, 250.0),
    *F_END,
]

RFlex_list = [
    TRAJ_ID_RFLEX,
    *fvec(0,  2.5,  0.0, 200.0),
    *fvec(1,  1.5, 20.0, 300.0),
    *F_END,
]

RExt_list = [
    TRAJ_ID_REXT,
    *fvec(0,  -1.25,  0.0, 200.0),
    *fvec(1,  -3.75, 15.0, 250.0),
    *F_END,
]
none_list = [
    TRAJ_ID_NONE,
    *fvec(0,  0,  0.0, 0.0),
    *fvec(0,  0, 0.0, 0.0),
    *F_END,
]

Left_Flex_Full_List = LFlex_list + none_list + none_list + none_list
Right_Flex_Full_List = RFlex_list + none_list + none_list + none_list
Right_Ext_Full_List = RExt_list + none_list + none_list + none_list
Left_Ext_Full_List = LExt_list + none_list + none_list + none_list

###########################################################################
# ============== SavingData_t 구조체 & CSV 포맷 (XM ↔ PC) ================
###########################################################################

FULL_STRUCT_FMT = '<HHIBBB58fH'
PACKET_SIZE     = struct.calcsize(FULL_STRUCT_FMT)  # 현재 245 bytes

CSV_HEADER = (
    "LoopCnt,H10Mode,H10AssistLevel,SmartAssist,"
    "LeftHipAngle,RightHipAngle,LeftThighAngle,RightThighAngle,"
    "LeftHipTorque,RightHipTorque,LeftHipMotorAngle,RightHipMotorAngle,"
    "LeftHipImuGlobalAccX,LeftHipImuGlobalAccY,LeftHipImuGlobalAccZ,"
    "LeftHipImuGlobalGyrX,LeftHipImuGlobalGyrY,LeftHipImuGlobalGyrZ,"
    "RightHipImuGlobalAccX,RightHipImuGlobalAccY,RightHipImuGlobalAccZ,"
    "RightHipImuGlobalGyrX,RightHipImuGlobalGyrY,RightHipImuGlobalGyrZ,"
    "TrunkIMU_LocalAccX,TrunkIMU_LocalAccY,TrunkIMU_LocalAccZ,"
    "TrunkIMU_LocalGyrX,TrunkIMU_LocalGyrY,TrunkIMU_LocalGyrZ,"
    "TrunkIMU_QuatW,TrunkIMU_QuatX,TrunkIMU_QuatY,TrunkIMU_QuatZ,"
    "LeftFSR1,LeftFSR2,LeftFSR3,LeftFSR4,LeftFSR5,LeftFSR6,"
    "LeftFSR7,LeftFSR8,LeftFSR9,LeftFSR10,LeftFSR11,LeftFSR12,"
    "LeftFSR13,LeftFSR14,"
    "RightFSR1,RightFSR2,RightFSR3,RightFSR4,RightFSR5,RightFSR6,"
    "RightFSR7,RightFSR8,RightFSR9,RightFSR10,RightFSR11,RightFSR12,"
    "RightFSR13,RightFSR14\n"
)

###########################################################################
# ============================== CRC16 함수 ===============================
###########################################################################

def crc16_modbus(data: bytes, init_val: int = 0xFFFF) -> int:
    crc = init_val
    for d in data:
        crc ^= d
        for _ in range(8):
            if (crc & 1):
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
        crc &= 0xFFFF
    return crc

###########################################################################
# ============================ F-VECTOR 송신 ===============================
###########################################################################

def send_fvec(float_list):
    global ser
    if ser is None:
        print("[ERROR] serial not opened")
        return False

    payload = b''.join(struct.pack('<f', v) for v in float_list)
    length  = len(payload)

    frame  = bytearray()
    frame += struct.pack('<H', SOF_VALUE)
    frame += struct.pack('<H', length)
    frame += payload

    crc    = crc16_modbus(frame[2:2+2+length])
    frame += struct.pack('<H', crc)

    ser.write(frame)
    print(f"[TX] Sent F-vector frame ({len(float_list)} floats, {len(frame)} bytes)")
    return True

###########################################################################
# ========================== 키보드 입력 스레드 ===========================
###########################################################################

def keyboard_thread_func():
    """
    콘솔에서 명령어를 받아 f-vector를 전송하거나 종료 플래그 설정.
    """
    global is_running

    print("\n[KEY] Keyboard control:")
    print("      'f' + Enter : send FULL F-vector (L/R Flex/Ext all)")
    print("      '1' + Enter : send LFlex only")
    print("      '2' + Enter : send LExt only")
    print("      '3' + Enter : send RFlex only")
    print("      '4' + Enter : send RExt only")
    print("      'q' + Enter : quit program\n")

    while is_running:
        try:
            cmd = input().strip().lower()
        except EOFError:
            # 터미널이 닫혔거나 입력 스트림 끊긴 경우
            break

        if not cmd:
            continue

        elif cmd == '1':
            print("[KEY] Sending LFlex f-vector...")
            send_fvec(Left_Flex_Full_List)
        elif cmd == '2':
            print("[KEY] Sending LExt f-vector...")
            send_fvec(Left_Ext_Full_List)
        elif cmd == '3':
            print("[KEY] Sending RFlex f-vector...")
            send_fvec(Right_Flex_Full_List)
        elif cmd == '4':
            print("[KEY] Sending RExt f-vector...")
            send_fvec(Right_Ext_Full_List)
        elif cmd == 'q':
            print("[KEY] Quit command received.")
            is_running = False
            break
        else:
            print("[KEY] Unknown command. Use 1 (Left Flexion) / 2 (Left Extension) / 3 (Right Flexion) / 4 (Right Extension) / q (quit)")

    print("[KEY] keyboard_thread_func terminated")

###########################################################################
# ============================ 데이터 수신 스레드 ==========================
###########################################################################

def rx_thread_func():
    global is_running, ser

    if ser is None:
        print("[ERROR] serial not opened in rx_thread")
        return

    print(f"[RX] Serial Connected: {SERIAL_PORT}")
    print("[RX] Sending 'AGRB MON START'")
    try:
        ser.write(b"AGRB MON START")
    except Exception as e:
        print(f"[RX] Failed to send MON START: {e}")

    buf = bytearray()

    try:
        while is_running:
            try:
                chunk = ser.read(512)
            except serial.SerialException as e:
                print(f"[RX] SerialException: {e}")
                break

            if not chunk:
                continue

            buf.extend(chunk)

            while True:
                if len(buf) < 4:
                    break

                sof_idx = -1
                for i in range(len(buf) - 1):
                    if buf[i] == (SOF_VALUE & 0xFF) and buf[i+1] == (SOF_VALUE >> 8):
                        sof_idx = i
                        break

                if sof_idx < 0:
                    buf = buf[-1:]
                    break

                if sof_idx > 0:
                    del buf[:sof_idx]

                if len(buf) < PACKET_SIZE:
                    break

                packet = bytes(buf[:PACKET_SIZE])

                length_field = int.from_bytes(packet[2:4], 'little')
                if length_field != PACKET_SIZE:
                    del buf[0]
                    continue

                data_without_crc = packet[:-2]
                recv_crc = int.from_bytes(packet[-2:], 'little')
                calc_crc = crc16_modbus(data_without_crc)

                if recv_crc != calc_crc:
                    del buf[0]
                    continue

                try:
                    data_queue.put(packet, timeout=0.01)
                except queue.Full:
                    # 큐가 꽉 차면 그냥 버림
                    pass

                del buf[:PACKET_SIZE]

    finally:
        print("[RX] rx_thread_func terminated")

###########################################################################
# =================== MAIN (TX + RX + CSV LOGGING) =======================
###########################################################################

def main():
    global ser, is_running

    print("--- CDC F-vector TX + SavingData RX Logger ---")
    print(f"PACKET_SIZE (SavingData_t) = {PACKET_SIZE} bytes")

    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT_SEC)
    print("[INFO] Serial Opened")

    # RX 스레드 시작
    t_rx = threading.Thread(target=rx_thread_func, daemon=True)
    t_rx.start()

    # 키보드 스레드 시작
    t_key = threading.Thread(target=keyboard_thread_func, daemon=True)
    t_key.start()

    # 더 이상 main에서 초기 f-vector는 보내지 않음
    # (키보드 명령으로만 전송)
    output_filename = "cdc_monitoring_data_log.csv"
    with open(output_filename, "w", encoding="utf-8", newline='') as f:
        f.write(CSV_HEADER)
        print(f"[INFO] Logging to: {output_filename}")
        print("[INFO] Receiving data... (Ctrl+C or 'q'+Enter to stop)")

        total_count   = 0
        pending_lines = []

        try:
            while is_running:
                try:
                    packet = data_queue.get(timeout=1.0)
                except queue.Empty:
                    # 1초 동안 새 패킷 없으면, 종료 플래그 확인 후 계속
                    continue

                try:
                    full = struct.unpack(FULL_STRUCT_FMT, packet)
                except struct.error as e:
                    print(f"[WARN] struct.unpack error: {e}")
                    continue

                data = full[2:-1]  # loopCnt ~ RightFSR14

                loop_cnt     = data[0]
                h10_mode     = data[1]
                h10_assist   = data[2]
                smart_assist = data[3]

                csv_line = [
                    str(loop_cnt),
                    str(h10_mode),
                    str(h10_assist),
                    str(smart_assist),
                ]

                for val in data[4:]:
                    csv_line.append(f"{val:.4f}")

                pending_lines.append(",".join(csv_line) + "\n")
                total_count += 1

                if total_count % 1000 == 0:
                    print(f"[INFO] received {total_count} packets (LoopCnt={loop_cnt})")

                if len(pending_lines) >= FLUSH_EVERY:
                    f.writelines(pending_lines)
                    pending_lines.clear()

        except KeyboardInterrupt:
            print("\n[INFO] KeyboardInterrupt; stopping...")
            is_running = False

        if pending_lines:
            f.writelines(pending_lines)

    # 종료 처리
    is_running = False
    try:
        print("[INFO] Sending 'AGRB MON STOP'")
        ser.write(b"AGRB MON STOP")
        time.sleep(0.1)
    except Exception:
        pass

    if ser and ser.is_open:
        ser.close()
        print("[INFO] Serial closed")

    t_rx.join(timeout=1.0)
    print(f"[INFO] Finished. Total {total_count} packets logged.")


if __name__ == "__main__":
    main()
