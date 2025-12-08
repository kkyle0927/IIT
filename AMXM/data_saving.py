import serial
import threading
import queue
import struct
import time
import os

# --- 1. 통신 설정 (Configuration) ---
SERIAL_PORT = 'COM4'   # 장치관리자에서 확인된 포트 번호
BAUD_RATE   = 921600   # USB-CDC에서는 실제 속도와 무관하지만 설정 필요
TIMEOUT_SEC = 1        # 읽기 타임아웃

# 2ms, 패킷 약 300 bytes라고 가정하면
# 1초에 약 500패킷 → 500 * 300B = 150kB/s 정도
# ↓ 1초마다 한 번씩 디스크에 flush 하도록 설정
FLUSH_EVERY = 500      # CSV 라인 500개 모을 때마다 한 번에 파일에 쓰기

# --- 2. SavingData_t 구조체와 동일한 패킷 정의 ---
# C 코드:
# typedef struct __attribute__((packed)) {
#     uint16_t sof;      // Start-of-frame (예: 0xAA55)
#     uint16_t len;      // 이 패킷 전체 길이 (sizeof(SavingData_t))
#
#     uint32_t loopCnt;
#     uint8_t  h10Mode;
#     uint8_t  h10AssistLevel;
#     uint8_t  SmartAssist;
#
#     float leftHipAngle,  rightHipAngle;
#     float leftThighAngle, rightThighAngle;
#     float leftHipTorque,  rightHipTorque;
#     float leftHipMotorAngle, rightHipMotorAngle;
#
#     float leftHipImuGlobalAccX, leftHipImuGlobalAccY, leftHipImuGlobalAccZ;
#     float leftHipImuGlobalGyrX, leftHipImuGlobalGyrY, leftHipImuGlobalGyrZ;
#     float rightHipImuGlobalAccX, rightHipImuGlobalAccY, rightHipImuGlobalAccZ;
#     float rightHipImuGlobalGyrX, rightHipImuGlobalGyrY, rightHipImuGlobalGyrZ;
#
#     float TrunkIMU_LocalAccX, TrunkIMU_LocalAccY, TrunkIMU_LocalAccZ;
#     float TrunkIMU_LocalGyrX, TrunkIMU_LocalGyrY, TrunkIMU_LocalGyrZ;
#     float TrunkIMU_QuatW, TrunkIMU_QuatX, TrunkIMU_QuatY, TrunkIMU_QuatZ;
#
#     float leftFSR1, leftFSR2, leftFSR3, leftFSR4, leftFSR5, leftFSR6;
#     float leftFSR7, leftFSR8, leftFSR9, leftFSR10, leftFSR11, leftFSR12;
#     float leftFSR13, leftFSR14;
#
#     float rightFSR1, rightFSR2, rightFSR3, rightFSR4, rightFSR5, rightFSR6;
#     float rightFSR7, rightFSR8, rightFSR9, rightFSR10, rightFSR11, rightFSR12;
#     float rightFSR13, rightFSR14;
#
#     uint16_t crc;      // sof~(crc 바로 앞까지)에 대한 CRC16
# } SavingData_t;

SOF_VALUE = 0xAA55  # C 쪽에서 사용한 값과 동일해야 함

# Python struct 포맷 (SavingData_t 전체)
# <   : little endian
# H   : uint16_t (sof)
# H   : uint16_t (len)
# I   : uint32_t (loopCnt)
# B   : uint8_t  (h10Mode)
# B   : uint8_t  (h10AssistLevel)
# B   : uint8_t  (SmartAssist)
# 58f : float * 58개
# H   : uint16_t (crc16)
FULL_STRUCT_FMT = '<HHIBBB58fH'
PACKET_SIZE = struct.calcsize(FULL_STRUCT_FMT)

# 유효 데이터 부분만 (loopCnt~RightFSR14): sof,len,crc 제외
# -> I (loopCnt) + B + B + B + 58f = 총 62 필드
DATA_STRUCT_FMT = '<IBBB58f'  # 참고용(언팩할 때는 FULL_STRUCT_FMT 다 쓰고 슬라이싱해도 됨)

# CSV 헤더 (데이터 순서대로: loopCnt~RightFSR14)
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

# 스레드 간 데이터 공유를 위한 큐 (여기에는 "정상 패킷 전체 bytes"를 넣음)
data_queue = queue.Queue()
is_running = True


# --- 3. CRC16 함수 (MODBUS 방식, poly=0xA001, init=0xFFFF) ---
def crc16_modbus(data: bytes, init_val: int = 0xFFFF) -> int:
    """
    C 코드의 CalcCrc16과 동일한 CRC16 계산:
    uint16_t CalcCrc16(const uint8_t* data, uint32_t length)
    {
        uint16_t crc = 0xFFFF;

        for (uint32_t i = 0; i < length; i++)
        {
            crc ^= data[i];
            for (uint8_t j = 0; j < 8; j++)
            {
                if (crc & 0x0001)
                    crc = (crc >> 1) ^ 0xA001;
                else
                    crc >>= 1;
            }
        }
        return crc;
    }
    """
    crc = init_val
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
            crc &= 0xFFFF
    return crc


# --- 4. 수신 스레드: 스트림에서 SOF/CRC 검사하며 완전한 패킷만 큐에 넣기 ---
def rx_thread_func():
    """
    [수신 스레드]
    - 시리얼에서 raw 바이트 스트림을 받아
    - SOF(0xAA55) + len + crc16까지 검사한 뒤
    - 정상 패킷만 data_queue에 넣는다.
    """
    global is_running
    ser = None

    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT_SEC)
        print(f"시리얼 포트 연결됨: {SERIAL_PORT}")
        print(f"SavingData_t 패킷 크기: {PACKET_SIZE} Bytes")

        print("모니터링 시작 명령 전송 (AGRB MON START)")
        ser.write(b"AGRB MON START")

        buf = bytearray()

        while is_running:
            try:
                # 한 번에 비교적 크게 읽어서 효율 향상
                chunk = ser.read(512)
            except serial.SerialException:
                break

            if not chunk:
                continue

            buf.extend(chunk)

            # 가능한 한 많이 패킷 파싱
            while True:
                # 최소한 sof(2) + len(2) 만큼은 있어야 한다
                if len(buf) < 4:
                    break

                # SOF(0xAA55, little-endian이면 0x55,0xAA)가 나올 위치 탐색
                sof_index = -1
                for i in range(len(buf) - 1):
                    if buf[i] == (SOF_VALUE & 0xFF) and buf[i + 1] == (SOF_VALUE >> 8):
                        sof_index = i
                        break

                if sof_index < 0:
                    # SOF 전까지 데이터는 모두 버리고, 마지막 1바이트만 남겨둠
                    buf = buf[-1:]
                    break

                # SOF 이전 쓰레기 바이트 제거
                if sof_index > 0:
                    del buf[:sof_index]

                # 이제 buf[0:2]는 sof, buf[2:4]는 len
                if len(buf) < PACKET_SIZE:
                    # 아직 전체 패킷이 안 들어왔음
                    break

                # 전체 패킷 후보
                packet = bytes(buf[:PACKET_SIZE])

                # len 필드 확인 (buf[2:4] little-endian)
                length_field = int.from_bytes(packet[2:4], byteorder='little')
                if length_field != PACKET_SIZE:
                    # 잘못된 헤더로 판단 → 한 바이트만 버리고 다시 탐색
                    # (혹시 우연히 0x55,0xAA가 끼어든 경우 대비)
                    # print(f"[WARN] length mismatch: {length_field} != {PACKET_SIZE}")
                    del buf[0]
                    continue

                # CRC 검사: sof~(crc 바로 앞까지)에 대한 CRC16
                data_without_crc = packet[:-2]
                recv_crc = int.from_bytes(packet[-2:], byteorder='little')
                calc_crc = crc16_modbus(data_without_crc)

                if recv_crc != calc_crc:
                    # CRC 불일치 → 이 헤더도 잘못된 것으로 간주, 1바이트 버리고 재시도
                    # print(f"[WARN] CRC mismatch: recv=0x{recv_crc:04X}, calc=0x{calc_crc:04X}")
                    del buf[0]
                    continue

                # 여기까지 왔으면 정상 패킷
                data_queue.put(packet)
                # 사용한 만큼 버퍼에서 제거
                del buf[:PACKET_SIZE]

    except Exception as e:
        print(f"통신 에러 발생: {e}")
        is_running = False

    finally:
        if ser and ser.is_open:
            print("\n모니터링 중지 명령 전송 (AGRB MON STOP)")
            try:
                ser.write(b"AGRB MON STOP")
                time.sleep(0.1)
            except Exception:
                pass
            ser.close()
            print("시리얼 포트 닫힘")


# --- 5. 메인 루프: 큐에서 정제된 패킷을 꺼내 CSV로 저장 ---
def main():
    global is_running

    print("--- USB CDC 실시간 데이터 로거 (프레이밍 + CRC 검증) 시작 ---")
    print(f"SavingData_t 구조체 크기(패킷 크기): {PACKET_SIZE} Bytes")
    print(f"CSV flush_every = {FLUSH_EVERY} 라인")

    # 수신 스레드 시작
    t = threading.Thread(target=rx_thread_func)
    t.daemon = True
    t.start()

    output_filename = "cdc_monitoring_data_log.csv"

    with open(output_filename, "w", encoding='utf-8', newline='') as f:
        f.write(CSV_HEADER)
        print(f"저장 파일: {output_filename}")
        print("데이터 수신 대기 중... (Ctrl+C로 종료)")

        total_count = 0
        pending_lines = []  # 여기에 CSV 라인을 모았다가 FLUSH_EVERY마다 한 번에 기록

        try:
            while is_running:
                try:
                    packet = data_queue.get(timeout=1)
                except queue.Empty:
                    if not t.is_alive():
                        break
                    continue

                # --- 데이터 파싱 (Decoding) ---
                try:
                    # SavingData_t 전체 언팩
                    full = struct.unpack(FULL_STRUCT_FMT, packet)
                    # full 튜플 인덱스:
                    #  0: sof
                    #  1: len
                    #  2: loopCnt
                    #  3: h10Mode
                    #  4: h10AssistLevel
                    #  5: SmartAssist
                    #  6~63: float 58개
                    # 64: crc16
                    data = full[2:-1]  # loopCnt ~ RightFSR14 (총 62개)

                    # 1. 정수 4개
                    loop_cnt      = data[0]
                    h10_mode      = data[1]
                    h10_assist    = data[2]
                    smart_assist  = data[3]

                    csv_line = [
                        str(loop_cnt),
                        str(h10_mode),
                        str(h10_assist),
                        str(smart_assist),
                    ]

                    # 2. 나머지 float 58개
                    for val in data[4:]:
                        csv_line.append(f"{val:.4f}")

                    # 한 줄 문자열로 만들고, 우선 메모리 버퍼에만 저장
                    pending_lines.append(",".join(csv_line) + "\n")
                    total_count += 1

                    # 일정 개수마다 한 번씩 디스크에 기록
                    if len(pending_lines) >= FLUSH_EVERY:
                        f.writelines(pending_lines)
                        pending_lines.clear()

                    if total_count % 1000 == 0:
                        print(f". (수신: {total_count}개, LoopCnt: {loop_cnt})", end='', flush=True)

                except struct.error as e:
                    print(f"\n파싱 오류: {e}")
                    continue

        except KeyboardInterrupt:
            print("\n사용자 종료 요청")

        # 루프 종료 후, 버퍼에 남은 라인도 한 번 더 기록
        if pending_lines:
            f.writelines(pending_lines)

    is_running = False
    t.join()
    print(f"\n종료 완료. 총 {total_count}개의 데이터가 저장되었습니다.")


if __name__ == "__main__":
    main()
