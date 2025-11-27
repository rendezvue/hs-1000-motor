import can
import time
import struct
import math

# --- 설정 변수: 이 부분만 수정하세요 ---
CAN_INTERFACE = 'can0'
NODE_ID = 5
TARGET_RPM = 3000     # 목표 속도 (RPM)
ENCODER_CPR = 10000  # 엔코더 1회전 당 펄스 수 (10000 가정)

# **새로운 가속/감속 변수 (단위: RPM/s)**
TARGET_ACCEL_RPM_PER_SEC = 100.0  # 가속도: 200 RPM / 50 RPM/s = 4초
TARGET_DECEL_RPM_PER_SEC = 100.0  # 감속도: 200 RPM / 50 RPM/s = 4초
# -------------------------------------

# --- CANopen 통신 변수 ---
SDO_TX_ID = 0x600 + NODE_ID  # 0x605
NMT_ID = 0x000


def calculate_speed_data(rpm, cpr, accel_rps, decel_rps):
    """
    RPM 값을 counts/s로 변환하고, RPM/s 기반으로 가속도/감속도 값을 계산합니다.
    """
    # 1. Target Velocity (속도) 계산: counts/s
    counts_per_second = int((rpm * cpr) / 60)
    
    # 2. 가속도/감속도 스케일링 팩터 (ELD2-CAN 예시 기반)
    scaling_factor = cpr / 60.0 
    
    accel_value = int(accel_rps * scaling_factor)
    decel_value = int(decel_rps * scaling_factor)
    
    # 3. 4바이트 Little Endian 포맷으로 변환
    velocity_data = list(struct.pack('<I', counts_per_second))
    accel_data = list(struct.pack('<I', accel_value))
    decel_data = list(struct.pack('<I', decel_value))
    
    return velocity_data, accel_data, decel_data


# 목표 속도 및 가속/감속 값 계산
VELOCITY_DATA, ACCEL_DATA, DECEL_DATA = calculate_speed_data(
    TARGET_RPM, ENCODER_CPR, TARGET_ACCEL_RPM_PER_SEC, TARGET_DECEL_RPM_PER_SEC
)
TARGET_COUNTS_PER_S = int.from_bytes(VELOCITY_DATA, byteorder='little')

# 예상 감속 시간 (time.sleep에 사용)
EXPECTED_DECEL_TIME = TARGET_RPM / TARGET_DECEL_RPM_PER_SEC


# --- Helper Functions (생략) ---
def send_can_message(arbitration_id, data):
    data_padded = data + [0x00] * (8 - len(data))
    message = can.Message(arbitration_id=arbitration_id, data=data_padded, is_extended_id=False)
    try:
        bus.send(message)
    except can.exceptions.CanOperationError as e:
        print(f"CAN 메시지 전송 오류: {e}")
        exit()

def send_sdo_write(index, subindex, data, data_len):
    csid = {1: 0x2F, 2: 0x2B, 4: 0x23}.get(data_len)
    index_low = index & 0xFF
    index_high = (index >> 8) & 0xFF
    
    sdo_data = data
    payload = [csid, index_low, index_high, subindex] + sdo_data
    send_can_message(SDO_TX_ID, payload)


# --- Main Sequence ---

if __name__ == '__main__':
    try:
        bus = can.interface.Bus(channel=CAN_INTERFACE, bustype='socketcan')
        print(f"\n--- CAN 버스 ({CAN_INTERFACE}) 초기화 성공 ---")
        
        # 1-4. NMT 초기화, PDO 설정, Operational Enable (이전과 동일)
        print("\n[1-4] 초기화 및 Operational Enable (이전과 동일)")
        send_can_message(NMT_ID, [0x81, 0x00]); time.sleep(2.0) 
        send_sdo_write(0x1017, 0x00, [0xE8, 0x03], 2); time.sleep(0.2)
        send_sdo_write(0x1600, 0x00, [0x00], 1)
        send_sdo_write(0x1600, 0x01, [0x10, 0x00, 0x40, 0x60], 4) 
        send_sdo_write(0x1600, 0x02, [0x20, 0x00, 0xFF, 0x60], 4)
        send_sdo_write(0x1600, 0x00, [0x02], 1); time.sleep(0.1)
        send_sdo_write(0x1400, 0x01, [0x05, 0x02, 0x00, 0x00], 4)
        send_sdo_write(0x1400, 0x02, [0xFF], 1); time.sleep(0.1)
        send_can_message(NMT_ID, [0x01, NODE_ID]); time.sleep(1.0) 

        send_sdo_write(0x6040, 0x00, [0x80, 0x00], 2)
        send_sdo_write(0x6040, 0x00, [0x00, 0x00], 2)
        send_sdo_write(0x6040, 0x00, [0x06, 0x00], 2)
        send_sdo_write(0x6040, 0x00, [0x07, 0x00], 2)
        send_sdo_write(0x6040, 0x00, [0x0F, 0x00], 2)
        time.sleep(0.5)

        # 5. 운전 모드 및 속도/가속 설정
        print(f"[5] {TARGET_RPM} RPM 구동 시작 (가속도: {TARGET_ACCEL_RPM_PER_SEC} RPM/s)")
        send_sdo_write(0x6060, 0x00, [0x03], 1); time.sleep(0.1)

        # 5.2. Profile Acceleration (6083h) 설정
        send_sdo_write(0x6083, 0x00, ACCEL_DATA, 4); time.sleep(0.1)
        
        # 5.3. Profile Deceleration (6084h) 설정
        send_sdo_write(0x6084, 0x00, DECEL_DATA, 4); time.sleep(0.1)

        # 5.4. Target Velocity (60FFh), 모터 구동 시작
        send_sdo_write(0x60FF, 0x00, VELOCITY_DATA, 4)
        
        print(f"-> 모터가 {TARGET_RPM} RPM (예상)으로 {5}초간 회전합니다.")
        time.sleep(20) 

        # 6. 모터 정지 (Target Velocity 0) 및 비활성화
        print("[6] 모터 정지 (Target Velocity 0) 및 비활성화")
        
        # 6.1. Target Velocity를 0으로 설정하여 감속 시작 (Controlword는 0Fh 유지)
        print("6.1. Target Velocity를 0으로 설정하여 감속 시작...")
        zero_data = [0x00, 0x00, 0x00, 0x00]
        send_sdo_write(0x60FF, 0x00, zero_data, 4)
        
        # 6.2. 감속 시간만큼 대기
        print(f"6.2. 예상 감속 시간 {EXPECTED_DECEL_TIME:.1f}초 동안 감속 대기...")
        time.sleep(EXPECTED_DECEL_TIME + 1.0) # 감속 시간 + 안전 여유 1초

        # 6.3. Control word = 06h (최종 비활성화)
        print("6.3. 최종 비활성화 (06h) 명령 전송.")
        send_sdo_write(0x6040, 0x00, [0x06, 0x00], 2) 
        
        print("\n--- 전체 시퀀스 완료 ---")
        bus.shutdown()

    except can.exceptions.CanOperationError as e:
        print(f"\nCAN0 인터페이스를 찾을 수 없습니다. 설정 상태를 확인하십시오.")
    except Exception as e:
        print(f"\n스크립트 실행 중 오류 발생: {e}")
        try:
            bus.shutdown()
        except:
            pass