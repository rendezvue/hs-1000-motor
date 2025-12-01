import can
import time
import struct
import tkinter as tk
from tkinter import messagebox
import threading
import queue
import math

CAN_INTERFACE = 'can0'
NODE_ID = 5
ENCODER_CPR = 10000 # ⭐️ 실제 드라이브의 설정된 엔코더 분해능으로 변경 필요
MM_PER_REV = 10.0 # ⭐️ 실제 '모터 축 1회전당 이송 거리(mm)'로 변경 필요
EMCY_ID = 0x80 + NODE_ID

# SDO 폴링 주기 (20ms로 설정)
SDO_POLLING_INTERVAL_SEC = 0.02

# CANopen 통신 변수
SDO_TX_ID = 0x600 + NODE_ID
NMT_ID = 0x000

# 글로벌 CAN 통신 및 스레딩 변수
bus = None
error_queue = queue.Queue()
realtime_position_counts_queue = queue.Queue()
can_thread = None
position_reader_thread = None
running = False

# 위치 제어 변수
reference_position_counts = 0
ref_pos_var = None

# Limit 탐색 변수
current_limit_search = None # 'BOTTOM', 'TOP', or None
bottom_limit_mm = None
top_limit_mm = None
bottom_limit_var = None
top_limit_var = None
SLOW_SEARCH_RPM = 20.0
SEARCH_ACCEL_RPS = 100.0
LIMIT_REVERSE_DISTANCE_MM = 10.0 # Limit 감지 후 후퇴 거리 (10mm)
LIMIT_REVERSE_VELOCITY_RPM = 50.0 # Limit 감지 후 후퇴 속도 (50 RPM)

# GUI 실시간 표시 변수 및 최신 위치 저장 변수
current_position_var = None
last_known_position_counts = 0

# GUI 위젯 변수
entry_limit_target_displacement = None
entry_limit_pos_rpm = None
entry_limit_pos_accel = None


# 계산 헬퍼 함수
def calculate_accel_data(rpm_per_sec, cpr):
    scaling_factor = cpr / 60.0
    value = int(rpm_per_sec * scaling_factor)
    return list(struct.pack('<I', value))

def calculate_velocity_data(rpm, cpr):
    counts_per_second = int((rpm * cpr) / 60)
    return list(struct.pack('<i', counts_per_second))

def mm_to_encoder_counts(mm, mm_per_rev, cpr):
    """mm 단위를 엔코더 펄스 수로 변환합니다. (부호 포함)"""
    counts = (mm / mm_per_rev) * cpr
    return list(struct.pack('<i', int(counts)))


# --- 2. CAN 통신 Helper Functions ---

def send_can_message(arbitration_id, data):
    if not bus: return
    data_padded = data + [0x00] * (8 - len(data))
    message = can.Message(
        arbitration_id=arbitration_id,
        data=data_padded,
        is_extended_id=False
    )
    try:
        bus.send(message)
    except can.exceptions.CanOperationError as e:
        pass

def send_sdo_write(index, subindex, data, data_len):
    csid = {1: 0x2F, 2: 0x2B, 4: 0x23}.get(data_len)
    index_low = index & 0xFF
    index_high = (index >> 8) & 0xFF
    
    sdo_data = data
    payload = [csid, index_low, index_high, subindex] + sdo_data
    send_can_message(SDO_TX_ID, payload)

def read_current_position_sdo():
    """SDO Read를 통해 현재 위치(mm)를 동기적으로 읽어 반환합니다. (기준 위치 설정 시 사용)"""
    if not bus: return None
    
    # SDO Read 요청: Position Actual Value (0x6064:00)
    request_data = [0x40, 0x64, 0x60, 0x00]
    send_can_message(SDO_TX_ID, request_data)

    response_id = 0x580 + NODE_ID
    start_time = time.time()
    
    while time.time() - start_time < 0.5:
        try:
            msg = bus.recv(timeout=0.01)
            # SDO 응답 메시지 (0x580 + NodeID)와 SDO Read 성공 (0x43) 확인
            if msg and msg.arbitration_id == response_id and msg.data[0] == 0x43:
                position_counts = struct.unpack('<i', bytes(msg.data[4:8]))[0]
                position_mm = (position_counts / ENCODER_CPR) * MM_PER_REV
                return position_mm
            elif msg and msg.arbitration_id == response_id and msg.data[0] == 0x80:
                print("SDO Read Abort during position capture.")
                return None
        except:
            return None
    return None


# --- 3. 비동기 CAN 리스너 및 GUI 업데이트 ---

def position_reader_thread_func():
    """실시간 위치를 SDO Read로 주기적으로 읽어 큐에 넣는 스레드 (폴링 방식)"""
    global running
    read_position_request = can.Message(
        arbitration_id=SDO_TX_ID,
        data=[0x40, 0x64, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00],
        is_extended_id=False
    )
    
    while running:
        if bus:
            try:
                bus.send(read_position_request)
            except Exception as e:
                pass
        
        # SDO 폴링 주기 (20ms 마다 요청)
        time.sleep(SDO_POLLING_INTERVAL_SEC)


def can_listener_thread():
    global running
    sdo_response_id = 0x580 + NODE_ID

    while running:
        try:
            msg = bus.recv(timeout=0.01)
            
            if msg and msg.arbitration_id == EMCY_ID:
                error_code = struct.unpack('<H', bytes(msg.data[:2]))[0]
                if error_code != 0x0000:
                    error_register = msg.data[2]
                    error_queue.put((error_code, error_register, msg.data))

            elif msg and msg.arbitration_id == sdo_response_id and msg.data[0] == 0x43:
                position_counts = struct.unpack('<i', bytes(msg.data[4:8]))[0]
                realtime_position_counts_queue.put(position_counts)
                
        except AttributeError:
            break
        except Exception as e:
            print(f"리스너 스레드 오류: {e}")
            break

def update_realtime_position():
    """메인 스레드에서 큐를 확인하고 GUI를 업데이트합니다. (SDO 폴링 데이터 사용)"""
    global last_known_position_counts
    
    if running and not realtime_position_counts_queue.empty():
        # 큐의 모든 메시지를 처리하고 가장 최근의 위치만 사용
        position_counts = None
        while not realtime_position_counts_queue.empty():
            position_counts = realtime_position_counts_queue.get()
            
        if position_counts is not None:
            last_known_position_counts = position_counts
            
            position_mm = (position_counts / ENCODER_CPR) * MM_PER_REV
            current_position_var.set(f"{position_mm:.4f} mm")

    if running:
        # GUI 갱신 주기
        root.after(int(SDO_POLLING_INTERVAL_SEC * 1000), update_realtime_position)

def check_for_errors():
    """EMCY 오류를 감지하고, Limit 찾기 중이면 위치 저장 및 모터 상태를 복구하고 후퇴 이동을 실행합니다."""
    global current_limit_search, bottom_limit_mm, top_limit_mm, last_known_position_counts
    
    if not error_queue.empty():
        error_code, error_register, raw_data = error_queue.get()
        data_hex = ' '.join(f'{b:02X}' for b in raw_data)
        
        error_message = (
            f"CANopen Emergency Message (EMCY) 수신!\n\n"
            f"오류 코드 (EMCY Error Code): 0x{error_code:04X}\n"
            f"오류 레지스터 (Standard Error Reg): 0x{error_register:02X}\n" # ⭐️ 수정: F-string 포맷 오류 해결 (0x2X -> 02X)
            f"원시 데이터: {data_hex}\n\n"
            f"모터 드라이브에 오류가 발생했습니다."
        )
        
        is_limit_found = False
        found_limit_type = None # ⭐️ 수정: 감지된 Limit 종류를 임시 저장
        
        # Limit 찾기 중 오류 발생 시 위치 저장 (Limit 탐색 중이 아니더라도 EMCY는 뜰 수 있음)
        if current_limit_search in ['BOTTOM', 'TOP']:
            is_limit_found = True
            found_limit_type = current_limit_search # ⭐️ 수정: 현재 찾던 Limit를 임시 저장

            # 1. Quick Stop
            try: send_sdo_write(0x6040, 0x00, [0x02, 0x00], 2); time.sleep(0.1)
            except: pass
            
            # 2. Shutdown
            try: send_sdo_write(0x6040, 0x00, [0x06, 0x00], 2); time.sleep(0.1)
            except: pass
            
            # ⭐️ 안정화 지연 시간 (1초)
            time.sleep(1.0)
            
            # 위치 저장 (지연 후 갱신된 last_known_position_counts 사용)
            position_counts = last_known_position_counts
            position_mm = (position_counts / ENCODER_CPR) * MM_PER_REV
            
            if position_mm is not None:
                if found_limit_type == 'BOTTOM': # ⭐️ 임시 변수 사용
                    bottom_limit_mm = position_mm
                    bottom_limit_var.set(f"{position_mm:.4f} mm")
                    messagebox.showinfo("Limit 찾기 완료", f"Bottom Limit 저장: {position_mm:.4f} mm (안정화 후 검출 시점)")
                elif found_limit_type == 'TOP': # ⭐️ 임시 변수 사용
                    top_limit_mm = position_mm
                    top_limit_var.set(f"{position_mm:.4f} mm")
                    messagebox.showinfo("Limit 찾기 완료", f"Top Limit 저장: {position_mm:.4f} mm (안정화 후 검출 시점)")
            
            # current_limit_search 초기화는 후퇴 이동 후에 수행됨

        # 일반 오류 메시지 표시
        if not is_limit_found or error_code == 0x8311:
            messagebox.showerror("🚨 모터 오류 발생 🚨", error_message)
        
        # --- 에러 초기화 및 모터 재활성화 (0x8311 오류 방지 및 후퇴 로직 추가) ---
        try:
            # 3. Fault Reset (Controlword 0x80)
            send_sdo_write(0x6040, 0x00, [0x80, 0x00], 2); time.sleep(0.1)
            
            # 0x8311 방지: 현재 위치를 목표 위치(0x607A)로 설정하여 오차를 0으로 만듦
            target_data = list(struct.pack('<i', last_known_position_counts))
            send_sdo_write(0x607A, 0x00, target_data, 4); time.sleep(0.1)
            
            # 4. Ready to Switch On (Controlword 0x06)
            send_sdo_write(0x6040, 0x00, [0x06, 0x00], 2); time.sleep(0.1)
            
            # 5. Switch On (Controlword 0x07)
            send_sdo_write(0x6040, 0x00, [0x07, 0x00], 2); time.sleep(0.1)
            
            # 6. Operation Enable (Controlword 0x0F)
            send_sdo_write(0x6040, 0x00, [0x0F, 0x00], 2); time.sleep(0.1)
            
            print("모터 EMCY 오류 초기화 및 Operation Enable 상태로 복구 완료.")

            # --- Limit 찾기 완료 시 10mm 후퇴 이동 ---
            if is_limit_found:
                # 7. Profile Position Mode로 전환
                send_sdo_write(0x6060, 0x00, [0x01], 1); time.sleep(0.05)
                
                # 후퇴 목표 위치 계산
                target_mm = None
                
                # ⭐️⭐️⭐️ 수정된 로직 ⭐️⭐️⭐️
                # TOP을 찾았다면: 위치값을 증가시켜 후퇴 (P_top + 10mm)
                if found_limit_type == 'TOP':
                    target_mm = top_limit_mm + LIMIT_REVERSE_DISTANCE_MM # ⭐️ 수정
                # BOTTOM을 찾았다면: 위치값을 감소시켜 후퇴 (P_bottom - 10mm)
                elif found_limit_type == 'BOTTOM':
                    target_mm = bottom_limit_mm - LIMIT_REVERSE_DISTANCE_MM # ⭐️ 수정
                # ⭐️⭐️⭐️ ⭐️⭐️⭐️
                
                if target_mm is not None:
                    # 목표 위치(mm)를 카운트로 변환
                    target_counts = int((target_mm / MM_PER_REV) * ENCODER_CPR)
                    
                    # 모터에 속도 및 가속도 파라미터 재설정
                    velocity_data = calculate_velocity_data(LIMIT_REVERSE_VELOCITY_RPM, ENCODER_CPR)
                    accel_data = calculate_accel_data(SEARCH_ACCEL_RPS, ENCODER_CPR)
                    send_sdo_write(0x6081, 0x00, velocity_data, 4); time.sleep(0.05)
                    send_sdo_write(0x6083, 0x00, accel_data, 4); time.sleep(0.05)
                    send_sdo_write(0x6084, 0x00, accel_data, 4); time.sleep(0.05)
                    
                    # 목표 위치 설정 및 이동 시작 (Controlword 0x1F)
                    position_data = list(struct.pack('<i', target_counts))
                    send_sdo_write(0x607A, 0x00, position_data, 4); time.sleep(0.1)
                    send_sdo_write(0x6040, 0x00, [0x1F, 0x00], 2); time.sleep(0.1)
                    send_sdo_write(0x6040, 0x00, [0x0F, 0x00], 2); time.sleep(0.1)
                    
                    messagebox.showinfo("Limit 후퇴 이동 시작", f"Limit 지점({position_mm:.4f} mm)에서 {LIMIT_REVERSE_DISTANCE_MM} mm 떨어진 {target_mm:.4f} mm로 후퇴합니다.")
            
            # ⭐️⭐️ Limit 검색 플래그 초기화
            current_limit_search = None
            
        except Exception as e:
            print(f"오류 복구/후퇴 중 예외 발생: {e}")
            # 이 단계에서 오류가 나더라도 플래그는 초기화해야 다음 Limit 찾기를 할 수 있도록 추가
            current_limit_search = None
            pass

    if running:
        root.after(int(SDO_POLLING_INTERVAL_SEC * 1000), check_for_errors)


# --- 4. GUI 컨트롤러 함수 ---

def initialize_can_bus():
    """CAN 버스를 초기화하고 Operational 상태로 전환하며, SDO 폴링 스레드를 시작합니다."""
    global bus, can_thread, position_reader_thread, running
    
    try:
        if bus: bus.shutdown()
        
        bus = can.interface.Bus(channel=CAN_INTERFACE, bustype='socketcan')
        running = True
        
        # 1. 리스너 스레드 시작
        can_thread = threading.Thread(target=can_listener_thread, daemon=True)
        can_thread.start()
        
        # 2. SDO 폴링 스레드 시작 (실시간 위치 요청)
        position_reader_thread = threading.Thread(target=position_reader_thread_func, daemon=True)
        position_reader_thread.start()

        # GUI 갱신 주기 시작
        root.after(int(SDO_POLLING_INTERVAL_SEC * 1000), check_for_errors)
        root.after(int(SDO_POLLING_INTERVAL_SEC * 1000), update_realtime_position)
        
        # NMT Pre-Operational 상태로 전환
        send_can_message(NMT_ID, [0x81, 0x00]); time.sleep(2.0)
        send_sdo_write(0x1017, 0x00, [0xE8, 0x03], 2); time.sleep(0.2)
        
        # NMT Operational 상태로 전환
        send_can_message(NMT_ID, [0x01, NODE_ID]); time.sleep(1.0)
        
        # 모터 상태 초기화 시퀀스
        send_sdo_write(0x6040, 0x00, [0x80, 0x00], 2)
        send_sdo_write(0x6040, 0x00, [0x00, 0x00], 2)
        
        zero_data = [0x00, 0x00, 0x00, 0x00]
        send_sdo_write(0x6064, 0x00, zero_data, 4)
        time.sleep(0.1)
        
        send_sdo_write(0x6060, 0x00, [0x03], 1); time.sleep(0.1)
        send_sdo_write(0x6040, 0x00, [0x06, 0x00], 2)
        
        messagebox.showinfo("초기화 성공", "CAN 버스 연결 및 모터 준비 완료 (SDO 폴링 실시간 위치 활성화).")
    
    except Exception as e:
        messagebox.showerror("초기화 실패", f"CAN 버스 초기화 실패: {e}")
        running = False
        if can_thread and can_thread.is_alive(): can_thread.join(timeout=0.2)
        if position_reader_thread and position_reader_thread.is_alive(): position_reader_thread.join(timeout=0.2)
        if bus: bus.shutdown()
        bus = None

def set_reference_position():
    """SDO Read를 통해 Position Actual Value (6064h)를 읽고 GUI에 표시합니다."""
    position_mm = read_current_position_sdo()
    if position_mm is not None:
        global reference_position_counts
        reference_position_counts = int((position_mm / MM_PER_REV) * ENCODER_CPR)
        if ref_pos_var:
            ref_pos_var.set(f"{position_mm:.4f} mm")
        messagebox.showinfo("1. 기준 위치 설정 완료", f"현재 위치: {position_mm:.4f} mm를 기준으로 설정했습니다.")
    else:
        messagebox.showerror("읽기 오류", "현재 위치를 읽는 데 실패했습니다.")


def run_absolute_position_mode():
    """Profile Position Mode(0x01)로 설정하고 절대 위치 이동을 명령합니다. (Controlword 0x1F/0x0F 사용)"""
    if not bus:
        messagebox.showerror("CAN 오류", "먼저 'CAN 초기화' 버튼을 눌러 버스를 연결하세요.")
        return

    try:
        target_absolute_mm = float(entry_target_absolute.get())
        velocity_rpm = float(entry_pos_rpm.get())
        accel_rps = float(entry_pos_accel.get())
        
        send_sdo_write(0x6060, 0x00, [0x01], 1); time.sleep(0.05)
        send_sdo_write(0x6040, 0x00, [0x06, 0x00], 2)
        send_sdo_write(0x6040, 0x00, [0x07, 0x00], 2)
        send_sdo_write(0x6040, 0x00, [0x0F, 0x00], 2)
        time.sleep(0.5)
        
        position_data = mm_to_encoder_counts(target_absolute_mm, MM_PER_REV, ENCODER_CPR)
        velocity_data = calculate_velocity_data(velocity_rpm, ENCODER_CPR)
        accel_data = calculate_accel_data(accel_rps, ENCODER_CPR)
        
        send_sdo_write(0x6081, 0x00, velocity_data, 4); time.sleep(0.05)
        send_sdo_write(0x6083, 0x00, accel_data, 4); time.sleep(0.05)
        send_sdo_write(0x6084, 0x00, accel_data, 4); time.sleep(0.05)
        send_sdo_write(0x607A, 0x00, position_data, 4)

        send_sdo_write(0x6040, 0x00, [0x1F, 0x00], 2)
        send_sdo_write(0x6040, 0x00, [0x0F, 0x00], 2)
        
        messagebox.showinfo("5. 구동 명령 (절대 위치)", f"절대 위치 모드 시작: 목표 위치 {target_absolute_mm} mm로 이동")

    except ValueError:
        messagebox.showerror("입력 오류", "위치 모드: 모든 입력 값은 숫자여야 합니다.")
    except Exception as e:
        messagebox.showerror("구동 오류", f"위치 모드 명령 중 오류 발생: {e}")


def run_limit_based_position_mode():
    """Limit 기준 (Bottom=0mm)으로 상대 위치 이동을 명령합니다."""
    global bottom_limit_mm
    
    if not bus:
        messagebox.showerror("CAN 오류", "먼저 'CAN 초기화' 버튼을 눌러 버스를 연결하세요.")
        return
    
    if bottom_limit_mm is None:
        messagebox.showerror("Limit 오류", "Bottom Limit 값이 설정되지 않았습니다. 'Bottom 찾기'를 먼저 실행하세요.")
        return

    try:
        # 1. Limit 기준 상대 변위 (mm)
        target_displacement_mm = float(entry_limit_target_displacement.get())
        velocity_rpm = float(entry_limit_pos_rpm.get())
        accel_rps = float(entry_limit_pos_accel.get())
        
# 2. 최종 절대 목표 위치 계산 (Bottom Limit = 0mm 기준)
        # ⭐️⭐️⭐️ 수정된 로직 ⭐️⭐️⭐️
        # Top 변위(양수)를 입력하면 Bottom Limit 값에서 빼야 Top 방향으로 이동함.
        target_absolute_mm = bottom_limit_mm - target_displacement_mm # ⭐️ 수정
        # ⭐️⭐️⭐️ ⭐️⭐️⭐️
        # 3. 모터 상태 전환 및 파라미터 설정 (Absolute Position Mode 재활용)
        send_sdo_write(0x6060, 0x00, [0x01], 1); time.sleep(0.05)
        send_sdo_write(0x6040, 0x00, [0x06, 0x00], 2)
        send_sdo_write(0x6040, 0x00, [0x07, 0x00], 2)
        send_sdo_write(0x6040, 0x00, [0x0F, 0x00], 2)
        time.sleep(0.5)
        
        # 4. 데이터 변환 및 전송
        position_data = mm_to_encoder_counts(target_absolute_mm, MM_PER_REV, ENCODER_CPR)
        velocity_data = calculate_velocity_data(velocity_rpm, ENCODER_CPR)
        accel_data = calculate_accel_data(accel_rps, ENCODER_CPR)
        
        send_sdo_write(0x6081, 0x00, velocity_data, 4); time.sleep(0.05)
        send_sdo_write(0x6083, 0x00, accel_data, 4); time.sleep(0.05)
        send_sdo_write(0x6084, 0x00, accel_data, 4); time.sleep(0.05)
        send_sdo_write(0x607A, 0x00, position_data, 4)

        # 5. 구동 시작
        send_sdo_write(0x6040, 0x00, [0x1F, 0x00], 2)
        send_sdo_write(0x6040, 0x00, [0x0F, 0x00], 2)
        
        messagebox.showinfo("Limit 기준 위치 구동",
                            f"Limit 기준 {target_displacement_mm} mm (절대 위치: {target_absolute_mm:.4f} mm)로 이동")

    except ValueError:
        messagebox.showerror("입력 오류", "Limit 기준 위치 모드: 모든 입력 값은 숫자여야 합니다.")
    except Exception as e:
        messagebox.showerror("구동 오류", f"Limit 기준 위치 모드 명령 중 오류 발생: {e}")


def position_stop_motor():
    """6. 모터를 멈추는 명령을 보냅니다. (Disable Voltage -> Quick Stop)"""
    global current_limit_search
    
    if not bus:
        messagebox.showerror("CAN 오류", "먼저 'CAN 초기화' 버튼을 눌러 버스를 연결하세요.")
        return
    try:
        # 1. Quick Stop 명령 전송
        send_sdo_write(0x6040, 0x00, [0x02, 0x00], 2)
        time.sleep(0.1)
        
        # 2. Shutdown 명령 전송 (Ready to Switch On 상태로 복귀)
        send_sdo_write(0x6040, 0x00, [0x06, 0x00], 2)
        
        # 3. Limit 검색 플래그 초기화
        current_limit_search = None
        
        messagebox.showinfo("6. 정지 명령", "위치 이동 정지 명령 전송 (Quick Stop) 및 모터 상태 Reset 완료.")
    except Exception as e:
        messagebox.showerror("정지 오류", f"정지 명령 중 오류 발생: {e}")

def start_velocity_search(direction):
    """Bottom 또는 Top을 찾기 위해 모터를 해당 방향으로 느리게 구동 시작"""
    global current_limit_search
    
    if not bus:
        return messagebox.showerror("CAN 오류", "먼저 'CAN 초기화' 버튼을 눌러 버스를 연결하세요.")
    
    if current_limit_search is not None:
        return messagebox.showwarning("경고", "이미 Limit를 찾는 중입니다. 현재 동작이 완료되거나 '위치이동 정지' 버튼을 누른 후 다시 시도해주세요.")

    try:
        current_limit_search = direction
        
        # ⭐️⭐️ 수정: TOP은 양수(+), BOTTOM은 음수(-) 방향으로 구동 ⭐️⭐️
        target_rpm = -SLOW_SEARCH_RPM if direction == 'TOP' else SLOW_SEARCH_RPM
        
        send_sdo_write(0x6060, 0x00, [0x03], 1); time.sleep(0.05)
        
        send_sdo_write(0x6040, 0x00, [0x06, 0x00], 2)
        send_sdo_write(0x6040, 0x00, [0x07, 0x00], 2)
        send_sdo_write(0x6040, 0x00, [0x0F, 0x00], 2)
        time.sleep(0.5)

        velocity_data = calculate_velocity_data(target_rpm, ENCODER_CPR)
        accel_data = calculate_accel_data(SEARCH_ACCEL_RPS, ENCODER_CPR)

        send_sdo_write(0x6083, 0x00, accel_data, 4); time.sleep(0.05)
        send_sdo_write(0x6084, 0x00, accel_data, 4); time.sleep(0.05)
        send_sdo_write(0x60FF, 0x00, velocity_data, 4)
        
        messagebox.showinfo(f"{direction} 찾기 시작", f"{direction} Limit를 찾기 위해 모터를 {'+' if direction == 'TOP' else '-'} 방향으로 {SLOW_SEARCH_RPM} RPM으로 구동합니다. 오류 발생 시 정지 및 위치 저장됩니다.")

    except Exception as e:
        messagebox.showerror("구동 오류", f"Limit 찾기 명령 중 오류 발생: {e}")
        current_limit_search = None


# ⭐️ 버튼 기능 교체 (GUI 레이아웃과의 연결을 명확히 함) ⭐️
def find_top_limit():
    start_velocity_search('TOP')
def find_bottom_limit():
    start_velocity_search('BOTTOM')


def on_closing():
    global bus, running
    
    running = False
    if can_thread and can_thread.is_alive():
        can_thread.join(timeout=0.2)
    if position_reader_thread and position_reader_thread.is_alive():
        position_reader_thread.join(timeout=0.2)

    if bus:
        try:
            send_sdo_write(0x6040, 0x00, [0x06, 0x00], 2)
            bus.shutdown()
        except:
            pass
    root.destroy()


# --- 5. Tkinter GUI 레이아웃 ---

root = tk.Tk()
root.title("ELD2-CAN 모터 제어 GUI (SDO 폴링)")
root.geometry("500x850") # GUI 높이 확장

btn_init = tk.Button(root, text="CAN 초기화 및 모터 준비", command=initialize_can_bus, bg='lightblue')
btn_init.pack(pady=10, padx=20, fill='x')

# --- 실시간 위치 표시 필드 ---
realtime_frame = tk.Frame(root, bd=1, relief=tk.RIDGE)
realtime_frame.pack(pady=5, padx=20, fill='x')
tk.Label(realtime_frame, text="🟢 현재 위치 (실시간 mm):", font=('Helvetica', 10, 'bold')).pack(side='left', padx=10, pady=5)
current_position_var = tk.StringVar()
current_position_var.set("N/A")
entry_current_pos = tk.Entry(realtime_frame, textvariable=current_position_var, state='readonly', width=15, font=('Helvetica', 12, 'bold'), fg='blue')
entry_current_pos.pack(side='right', padx=10, pady=5)
# ----------------------------------------

# -----------------
# [섹션 0: Limit 자동 찾기]
# -----------------
tk.Label(root, text="--- 0. Limit 자동 찾기 (Over Current 검출) ---", font=('Helvetica', 10, 'bold')).pack(pady=5)
limit_frame = tk.Frame(root, bd=1, relief=tk.SOLID, padx=10, pady=5)
limit_frame.pack(pady=5, padx=20, fill='x')

# ⭐️ Top Limit 찾기 (Top 버튼 연결)
bottom_frame = tk.Frame(limit_frame)
bottom_frame.pack(pady=5, fill='x')
# command=find_top_limit로 연결
btn_find_top_new = tk.Button(bottom_frame, text="▶ Top 찾기 (+)", command=find_top_limit, bg='#FFD2D2')
btn_find_top_new.pack(side='left', padx=5, pady=2)
tk.Label(bottom_frame, text="Top Limit (mm):").pack(side='left', padx=5, pady=2)
top_limit_var = tk.StringVar()
top_limit_var.set("N/A")
entry_top_limit = tk.Entry(bottom_frame, textvariable=top_limit_var, state='readonly', width=15)
entry_top_limit.pack(side='left', expand=True, fill='x', padx=5, pady=2)

# ⭐️ Bottom Limit 찾기 (Bottom 버튼 연결)
top_frame = tk.Frame(limit_frame)
top_frame.pack(pady=5, fill='x')
# command=find_bottom_limit로 연결
btn_find_bottom_new = tk.Button(top_frame, text="▶ Bottom 찾기 (-)", command=find_bottom_limit, bg='#D2FFD2')
btn_find_bottom_new.pack(side='left', padx=5, pady=2)
tk.Label(top_frame, text="Bottom Limit (mm):").pack(side='left', padx=5, pady=2)
bottom_limit_var = tk.StringVar()
bottom_limit_var.set("N/A")
entry_bottom_limit = tk.Entry(top_frame, textvariable=bottom_limit_var, state='readonly', width=15)
entry_bottom_limit.pack(side='left', expand=True, fill='x', padx=5, pady=2)


# -----------------
# [섹션 1: 절대 위치 제어]
# -----------------
tk.Label(root, text="--- 1. 절대 위치 제어 (Absolute Position Mode) ---", font=('Helvetica', 10, 'bold')).pack(pady=10)
position_frame = tk.Frame(root, bd=1, relief=tk.SOLID)
position_frame.pack(pady=5, padx=20, fill='x')

ref_frame = tk.Frame(position_frame)
ref_frame.pack(pady=5, padx=5, fill='x')
btn_set_ref = tk.Button(ref_frame, text="1. 현재 위치 기준 설정", command=set_reference_position, bg='yellow')
btn_set_ref.pack(side='left', padx=5, pady=2)
tk.Label(ref_frame, text="기준 위치 (mm):").pack(side='left', padx=5, pady=2)
ref_pos_var = tk.StringVar()
ref_pos_var.set("0.00 mm")
entry_ref_pos = tk.Entry(ref_frame, textvariable=ref_pos_var, state='readonly', width=15)
entry_ref_pos.pack(side='left', expand=True, fill='x', padx=5, pady=2)


input_frame_pos = tk.Frame(position_frame)
input_frame_pos.pack(pady=5, padx=5)

tk.Label(input_frame_pos, text="2. 목표 Absolute Position (mm)").grid(row=0, column=0, padx=5, pady=2, sticky='w')
entry_target_absolute = tk.Entry(input_frame_pos)
entry_target_absolute.insert(0, "-580.0")
entry_target_absolute.grid(row=0, column=1, padx=5, pady=2)

tk.Label(input_frame_pos, text="3. 속도 RPM").grid(row=1, column=0, padx=5, pady=2, sticky='w')
entry_pos_rpm = tk.Entry(input_frame_pos)
entry_pos_rpm.insert(0, "100")
entry_pos_rpm.grid(row=1, column=1, padx=5, pady=2)

tk.Label(input_frame_pos, text="4. 가속도 RPM/s").grid(row=2, column=0, padx=5, pady=2, sticky='w')
entry_pos_accel = tk.Entry(input_frame_pos)
entry_pos_accel.insert(0, "200.0")
entry_pos_accel.grid(row=2, column=1, padx=5, pady=2)

button_frame_pos = tk.Frame(position_frame)
button_frame_pos.pack(pady=5, padx=5, fill='x')

btn_pos_run = tk.Button(button_frame_pos, text="5. ▶ 위치이동 구동", command=run_absolute_position_mode, bg='lightgreen')
btn_pos_run.pack(side='left', expand=True, fill='x', padx=5)

btn_pos_stop = tk.Button(button_frame_pos, text="6. ■ 위치이동 정지", command=position_stop_motor, bg='salmon')
btn_pos_stop.pack(side='right', expand=True, fill='x', padx=5)


# -----------------
# ⭐️ [섹션 2: Limit 기준 위치 제어 (Bottom=0mm으로 재변경)] ⭐️
# -----------------
tk.Label(root, text="--- 2. Limit 기준 위치 제어 (Bottom=0mm) ---", font=('Helvetica', 10, 'bold')).pack(pady=10)
limit_position_frame = tk.Frame(root, bd=1, relief=tk.SOLID)
limit_position_frame.pack(pady=5, padx=20, fill='x')

limit_input_frame = tk.Frame(limit_position_frame)
limit_input_frame.pack(pady=5, padx=5)

tk.Label(limit_input_frame, text="1. 목표 Top 변위 (mm)").grid(row=0, column=0, padx=5, pady=2, sticky='w')
entry_limit_target_displacement = tk.Entry(limit_input_frame)
entry_limit_target_displacement.insert(0, "10.0")
entry_limit_target_displacement.grid(row=0, column=1, padx=5, pady=2)

tk.Label(limit_input_frame, text="2. 속도 RPM").grid(row=1, column=0, padx=5, pady=2, sticky='w')
entry_limit_pos_rpm = tk.Entry(limit_input_frame)
entry_limit_pos_rpm.insert(0, "100")
entry_limit_pos_rpm.grid(row=1, column=1, padx=5, pady=2)

tk.Label(limit_input_frame, text="3. 가속도 RPM/s").grid(row=2, column=0, padx=5, pady=2, sticky='w')
entry_limit_pos_accel = tk.Entry(limit_input_frame)
entry_limit_pos_accel.insert(0, "200.0")
entry_limit_pos_accel.grid(row=2, column=1, padx=5, pady=2)

limit_button_frame = tk.Frame(limit_position_frame)
limit_button_frame.pack(pady=5, padx=5, fill='x')

btn_limit_pos_run = tk.Button(limit_button_frame, text="4. ▶ Limit기준 위치 구동", command=run_limit_based_position_mode, bg='#FFE0B2')
btn_limit_pos_run.pack(side='left', expand=True, fill='x', padx=5)

btn_limit_pos_stop = tk.Button(limit_button_frame, text="5. ■ 위치이동 정지", command=position_stop_motor, bg='salmon')
btn_limit_pos_stop.pack(side='right', expand=True, fill='x', padx=5)


root.protocol("WM_DELETE_WINDOW", on_closing)

root.mainloop()