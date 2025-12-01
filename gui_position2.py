import can
import time
import struct
import tkinter as tk
from tkinter import messagebox
import threading
import queue
import math

# --- 1. 설정 변수 및 계산 함수 ---
CAN_INTERFACE = 'can0'
NODE_ID = 5
ENCODER_CPR = 10000  # 엔코더 1회전 당 펄스 수 (10000 가정)
MM_PER_REV = 10.0 # 모터 1회전당 이동 거리 (mm) - 예시값
EMCY_ID = 0x80 + NODE_ID 

# CANopen 통신 변수
SDO_TX_ID = 0x600 + NODE_ID  
NMT_ID = 0x000

# 글로벌 CAN 통신 및 스레딩 변수
bus = None
error_queue = queue.Queue()  
can_thread = None
running = False 

# 상대 위치 기준점 (참고용)
reference_position_counts = 0 
ref_pos_var = None # Tkinter StringVar for GUI update

# 계산 헬퍼 함수 (이전과 동일)
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


# --- 2. CAN 통신 Helper Functions (이전과 동일) ---

def send_can_message(arbitration_id, data):
    if not bus:
        messagebox.showerror("CAN 오류", "CAN 버스가 초기화되지 않았습니다.")
        return
    data_padded = data + [0x00] * (8 - len(data))
    message = can.Message(
        arbitration_id=arbitration_id,
        data=data_padded,
        is_extended_id=False
    )
    try:
        bus.send(message)
    except can.exceptions.CanOperationError as e:
        messagebox.showerror("CAN 전송 오류", f"메시지 전송 실패: {e}")
        return

def send_sdo_write(index, subindex, data, data_len):
    csid = {1: 0x2F, 2: 0x2B, 4: 0x23}.get(data_len)
    index_low = index & 0xFF
    index_high = (index >> 8) & 0xFF
    
    sdo_data = data
    payload = [csid, index_low, index_high, subindex] + sdo_data
    send_can_message(SDO_TX_ID, payload)


# --- 3. 비동기 CAN 리스너 및 GUI 업데이트 (이전과 동일) ---

def can_listener_thread():
    global running
    while running:
        try:
            msg = bus.recv(timeout=0.1) 
            
            if msg and msg.arbitration_id == EMCY_ID:
                error_code = struct.unpack('<H', bytes(msg.data[:2]))[0]
                
                if error_code != 0x0000:
                    error_register = msg.data[2]
                    error_queue.put((error_code, error_register, msg.data))
                
        except AttributeError:
            break
        except Exception as e:
            print(f"리스너 스레드 오류: {e}")
            break

def check_for_errors():
    if not error_queue.empty():
        error_code, error_register, raw_data = error_queue.get()
        data_hex = ' '.join(f'{b:02X}' for b in raw_data)
        
        error_message = (
            f"CANopen Emergency Message (EMCY) 수신!\n\n"
            f"오류 코드 (EMCY Error Code): 0x{error_code:04X}\n"
            f"오류 레지스터 (Standard Error Reg): 0x{error_register:02X}\n"
            f"원시 데이터: {data_hex}\n\n"
            f"모터 드라이브에 오류가 발생했습니다. 매뉴얼을 참조하십시오."
        )
        messagebox.showerror("🚨 모터 오류 발생 🚨", error_message)
        
        try:
            send_sdo_write(0x6040, 0x00, [0x06, 0x00], 2)
        except:
             pass

    if running:
        root.after(100, check_for_errors)


# --- 4. GUI 컨트롤러 함수 ---

def initialize_can_bus():
    """CAN 버스를 초기화하고 Operational 상태로 전환하며, 리스너 스레드를 시작합니다."""
    global bus, can_thread, running
    
    try:
        if bus: bus.shutdown()
        
        bus = can.interface.Bus(channel=CAN_INTERFACE, bustype='socketcan')
        running = True
        can_thread = threading.Thread(target=can_listener_thread, daemon=True)
        can_thread.start()
        root.after(100, check_for_errors)
        
        send_can_message(NMT_ID, [0x81, 0x00]); time.sleep(2.0) 
        send_sdo_write(0x1017, 0x00, [0xE8, 0x03], 2); time.sleep(0.2)
        send_sdo_write(0x1600, 0x00, [0x00], 1)
        send_sdo_write(0x1600, 0x01, [0x10, 0x00, 0x40, 0x60], 4) 
        send_sdo_write(0x1600, 0x02, [0x20, 0x00, 0xFF, 0x60], 4)
        send_sdo_write(0x1600, 0x00, [0x02], 1); time.sleep(0.1)
        send_sdo_write(0x1400, 0x01, [0x05, 0x02, 0x00, 0x00], 4)
        send_sdo_write(0x1400, 0x02, [0xFF], 1); time.sleep(0.1)
        send_can_message(NMT_ID, [0x01, NODE_ID]); time.sleep(1.0) 

        # Fault Reset 및 초기 상태 설정
        send_sdo_write(0x6040, 0x00, [0x80, 0x00], 2)
        send_sdo_write(0x6040, 0x00, [0x00, 0x00], 2)
        
        # Position Actual Value (6064h)를 0으로 강제 리셋 (Soft Homing 시도)
        zero_data = [0x00, 0x00, 0x00, 0x00]
        send_sdo_write(0x6064, 0x00, zero_data, 4) 
        time.sleep(0.1) 
        
        send_sdo_write(0x6060, 0x00, [0x03], 1); time.sleep(0.1) 
        send_sdo_write(0x6040, 0x00, [0x06, 0x00], 2) 
        
        messagebox.showinfo("초기화 성공", "CAN 버스 연결 및 리스너 시작, 모터 준비 완료.")
    
    except Exception as e:
        messagebox.showerror("초기화 실패", f"CAN 버스 초기화 실패: {e}")
        running = False
        if can_thread and can_thread.is_alive(): can_thread.join(timeout=0.2)
        if bus: bus.shutdown()
        bus = None

# --- [1. 현재 위치 읽기 (기준 설정)] ---
def set_reference_position():
    """SDO Read를 통해 Position Actual Value (6064h)를 읽고 GUI에 표시합니다."""
    global reference_position_counts
    if not bus:
        messagebox.showerror("CAN 오류", "먼저 'CAN 초기화' 버튼을 눌러 버스를 연결하세요.")
        return

    # 1. SDO Read Request (6064h Position Actual Value)
    request_data = [0x40, 0x64, 0x60, 0x00]
    send_can_message(SDO_TX_ID, request_data)

    try:
        # 2. SDO Read Response 수신 대기 (Timeout 1초)
        response_id = 0x580 + NODE_ID
        
        start_time = time.time()
        while time.time() - start_time < 1.0:
            msg = bus.recv(timeout=0.1)
            
            if msg is None: continue 
            
            if msg.arbitration_id == response_id:
                # 3. 응답 데이터 확인 및 디코딩 (CSID 0x43: 4-byte read confirmed)
                if msg.data[0] == 0x43: 
                    position_counts = struct.unpack('<i', bytes(msg.data[4:8]))[0]
                    
                    # Store it globally (for user reference)
                    reference_position_counts = position_counts 
                    
                    # 4. Counts를 mm로 변환 및 GUI 업데이트
                    position_mm = (position_counts / ENCODER_CPR) * MM_PER_REV
                    if ref_pos_var:
                        ref_pos_var.set(f"{position_mm:.4f} mm")
                    messagebox.showinfo("1. 기준 위치 설정 완료", f"현재 위치: {position_mm:.4f} mm를 기준으로 설정했습니다.")
                    return
                elif msg.data[0] == 0x80:
                    abort_code = struct.unpack('<I', bytes(msg.data[4:8]))[0]
                    messagebox.showerror("SDO 오류", f"위치 읽기 오류 발생. Abort Code: 0x{abort_code:08X}")
                    return
        
        messagebox.showerror("통신 오류", "SDO 응답 시간 초과.")

    except Exception as e:
        messagebox.showerror("읽기 오류", f"위치 데이터 읽기 중 오류 발생: {e}")


# --- [2. 절대 위치 제어 실행 (Absolute Position Mode)] ---
def run_absolute_position_mode():
    """Profile Position Mode(0x01)로 설정하고 절대 위치 이동을 명령합니다. (Controlword 0x1F/0x0F 사용)"""
    if not bus:
        messagebox.showerror("CAN 오류", "먼저 'CAN 초기화' 버튼을 눌러 버스를 연결하세요.")
        return

    try:
        # 2. 목표 Absolute Position (mm)
        target_absolute_mm = float(entry_target_absolute.get()) 
        # 3. 속도 RPM
        velocity_rpm = float(entry_pos_rpm.get())
        # 4. 가속도 RPM
        accel_rps = float(entry_pos_accel.get())
        
        # 0. Profile Position Mode (0x01) 설정
        send_sdo_write(0x6060, 0x00, [0x01], 1); time.sleep(0.05)
        
        # 1. 모터를 Operation Enable로 재활성화
        send_sdo_write(0x6040, 0x00, [0x06, 0x00], 2)
        send_sdo_write(0x6040, 0x00, [0x07, 0x00], 2)
        send_sdo_write(0x6040, 0x00, [0x0F, 0x00], 2)
        time.sleep(0.5)
        
        # 목표 위치(mm)를 엔코더 펄스로 변환하여 전송
        position_data = mm_to_encoder_counts(target_absolute_mm, MM_PER_REV, ENCODER_CPR)
        velocity_data = calculate_velocity_data(velocity_rpm, ENCODER_CPR)
        accel_data = calculate_accel_data(accel_rps, ENCODER_CPR)
        
        send_sdo_write(0x6081, 0x00, velocity_data, 4); time.sleep(0.05)
        send_sdo_write(0x6083, 0x00, accel_data, 4); time.sleep(0.05)
        send_sdo_write(0x6084, 0x00, accel_data, 4); time.sleep(0.05) 
        send_sdo_write(0x607A, 0x00, position_data, 4) 

        # 3. Controlword 전송 (Absolute Position Mode: 0x1F)
        send_sdo_write(0x6040, 0x00, [0x1F, 0x00], 2) 
        
        # 4. New Setpoint 비트 해제 (이동 시작)
        send_sdo_write(0x6040, 0x00, [0x0F, 0x00], 2) 
        
        messagebox.showinfo("5. 구동 명령 (절대 위치)", f"절대 위치 모드 시작: 목표 위치 {target_absolute_mm} mm로 이동 (Controlword 0x1F/0x0F)")

    except ValueError:
        messagebox.showerror("입력 오류", "위치 모드: 모든 입력 값은 숫자여야 합니다.")
    except Exception as e:
        messagebox.showerror("구동 오류", f"위치 모드 명령 중 오류 발생: {e}")

def position_stop_motor():
    """6. 모터를 멈추는 명령을 보냅니다. (Disable Voltage -> Quick Stop)"""
    if not bus:
        messagebox.showerror("CAN 오류", "먼저 'CAN 초기화' 버튼을 눌러 버스를 연결하세요.")
        return
    try:
        # Quick Stop (0x02)
        send_sdo_write(0x6040, 0x00, [0x02, 0x00], 2) 
        # Shutdown (0x06)
        send_sdo_write(0x6040, 0x00, [0x06, 0x00], 2) 
        messagebox.showinfo("6. 정지 명령", "위치 이동 정지 명령 전송 (Quick Stop)")
    except Exception as e:
        messagebox.showerror("정지 오류", f"정지 명령 중 오류 발생: {e}")


def on_closing():
    global bus, running
    
    running = False 
    if can_thread and can_thread.is_alive():
        can_thread.join(timeout=0.2) 

    if bus:
        try:
            send_sdo_write(0x6040, 0x00, [0x06, 0x00], 2)
            bus.shutdown()
        except:
            pass
    root.destroy()


# --- 5. Tkinter GUI 레이아웃 ---

root = tk.Tk()
root.title("ELD2-CAN 모터 제어 GUI")
root.geometry("500x480") 

btn_init = tk.Button(root, text="CAN 초기화 및 모터 준비", command=initialize_can_bus, bg='lightblue')
btn_init.pack(pady=10, padx=20, fill='x')

# -----------------
# [섹션 1: 절대 위치 제어] 
# -----------------
tk.Label(root, text="--- 1. 절대 위치 제어 (Absolute Position Mode) ---", font=('Helvetica', 10, 'bold')).pack(pady=10)
position_frame = tk.Frame(root, bd=1, relief=tk.SOLID)
position_frame.pack(pady=5, padx=20, fill='x')

# 기준 위치 설정 및 표시 (1번 요청)
ref_frame = tk.Frame(position_frame)
ref_frame.pack(pady=5, padx=5, fill='x')

btn_set_ref = tk.Button(ref_frame, text="1. 현재 위치 기준 설정", command=set_reference_position, bg='yellow')
btn_set_ref.pack(side='left', padx=5, pady=2)

tk.Label(ref_frame, text="현재 위치 (mm):").pack(side='left', padx=5, pady=2)
ref_pos_var = tk.StringVar()
ref_pos_var.set("0.00 mm")
entry_ref_pos = tk.Entry(ref_frame, textvariable=ref_pos_var, state='readonly', width=15)
entry_ref_pos.pack(side='left', expand=True, fill='x', padx=5, pady=2)


# 이동 설정 입력 필드 (2, 3, 4번 요청)
input_frame_pos = tk.Frame(position_frame)
input_frame_pos.pack(pady=5, padx=5)

tk.Label(input_frame_pos, text="2. 목표 Absolute Position (mm)").grid(row=0, column=0, padx=5, pady=2, sticky='w')
entry_target_absolute = tk.Entry(input_frame_pos)
entry_target_absolute.insert(0, "-580.0") # 절대 목표 위치
entry_target_absolute.grid(row=0, column=1, padx=5, pady=2)

tk.Label(input_frame_pos, text="3. 속도 RPM").grid(row=1, column=0, padx=5, pady=2, sticky='w')
entry_pos_rpm = tk.Entry(input_frame_pos)
entry_pos_rpm.insert(0, "100")
entry_pos_rpm.grid(row=1, column=1, padx=5, pady=2)

tk.Label(input_frame_pos, text="4. 가속도 RPM/s").grid(row=2, column=0, padx=5, pady=2, sticky='w')
entry_pos_accel = tk.Entry(input_frame_pos)
entry_pos_accel.insert(0, "200.0")
entry_pos_accel.grid(row=2, column=1, padx=5, pady=2)

# 구동/정지 버튼 (5, 6번 요청)
button_frame_pos = tk.Frame(position_frame)
button_frame_pos.pack(pady=5, padx=5, fill='x')

btn_pos_run = tk.Button(button_frame_pos, text="5. ▶ 위치이동 구동", command=run_absolute_position_mode, bg='lightgreen')
btn_pos_run.pack(side='left', expand=True, fill='x', padx=5)

btn_pos_stop = tk.Button(button_frame_pos, text="6. ■ 위치이동 정지", command=position_stop_motor, bg='salmon')
btn_pos_stop.pack(side='right', expand=True, fill='x', padx=5)


# 창 닫기 시 이벤트 처리
root.protocol("WM_DELETE_WINDOW", on_closing)

# GUI 실행
root.mainloop()