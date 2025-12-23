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
EMCY_ID = 0x80 + NODE_ID  # Node 5의 Emergency Object (EMCY) ID: 0x85

# CANopen 통신 변수
SDO_TX_ID = 0x600 + NODE_ID  # 0x605
NMT_ID = 0x000

# 글로벌 CAN 통신 및 스레딩 변수
bus = None
error_queue = queue.Queue()  
can_thread = None
running = False 

def calculate_accel_data(rpm_per_sec, cpr):
    """RPM/s 기반으로 가속도/감속도 값을 계산합니다. (항상 양수)"""
    scaling_factor = cpr / 60.0 
    value = int(rpm_per_sec * scaling_factor)
    return list(struct.pack('<I', value))

def calculate_velocity_data(rpm, cpr):
    """
    RPM 값을 counts/s (부호 포함)로 변환합니다.
    **포맷이 '<i' (Signed 32-bit Integer)로 변경되었습니다.**
    """
    # RPM에 따라 counts/s도 부호를 가집니다.
    counts_per_second = int((rpm * cpr) / 60)
    # '<i': Little Endian, Signed Integer (4 bytes)
    return list(struct.pack('<i', counts_per_second))


# --- 2. CAN 통신 Helper Functions (이전과 동일) ---

def send_can_message(arbitration_id, data):
    """지정된 ID와 데이터로 CAN 메시지를 전송합니다."""
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
    """SDO Write 명령을 전송합니다."""
    csid = {1: 0x2F, 2: 0x2B, 4: 0x23}.get(data_len)
    index_low = index & 0xFF
    index_high = (index >> 8) & 0xFF
    
    sdo_data = data
    payload = [csid, index_low, index_high, subindex] + sdo_data
    send_can_message(SDO_TX_ID, payload)


# --- 3. 비동기 CAN 리스너 및 GUI 업데이트 (이전과 동일) ---

def can_listener_thread():
    """백그라운드에서 CAN 메시지를 수신하고 EMCY 메시지를 큐에 추가합니다."""
    global running
    while running:
        try:
            msg = bus.recv(timeout=0.1) 
            
            if msg and msg.arbitration_id == EMCY_ID:
                error_code = struct.unpack('<H', bytes(msg.data[:2]))[0]
                error_register = msg.data[2]
                error_queue.put((error_code, error_register, msg.data))
                
        except AttributeError:
            break
        except Exception as e:
            print(f"리스너 스레드 오류: {e}")
            break

def check_for_errors():
    """메인 GUI 스레드에서 주기적으로 오류 큐를 확인하고 메시지를 표시합니다."""
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
        
        # 오류 발생 시 안전을 위해 모터 비활성화 명령 전송
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
        
        # 1. CAN 버스 연결 및 리스너 스레드 시작
        bus = can.interface.Bus(channel=CAN_INTERFACE, bustype='socketcan')
        running = True
        can_thread = threading.Thread(target=can_listener_thread, daemon=True)
        can_thread.start()
        root.after(100, check_for_errors)
        
        # 2. NMT 초기화, PDO 설정, DSP402 초기 상태 진입
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
        send_sdo_write(0x6060, 0x00, [0x03], 1); time.sleep(0.1)
        send_sdo_write(0x6040, 0x00, [0x06, 0x00], 2) 
        
        messagebox.showinfo("초기화 성공", "CAN 버스 연결 및 리스너 시작, 모터 준비 완료.")
    
    except Exception as e:
        messagebox.showerror("초기화 실패", f"CAN 버스 초기화 실패: {e}")
        running = False
        if can_thread and can_thread.is_alive(): can_thread.join(timeout=0.2)
        if bus: bus.shutdown()
        bus = None

def run_motor():
    """모터를 Operation Enable 상태로 전환하고 입력된 값으로 구동합니다."""
    if not bus:
        messagebox.showerror("CAN 오류", "먼저 'CAN 초기화' 버튼을 눌러 버스를 연결하세요.")
        return

    try:
        # 입력 값 읽기 (부호 포함)
        target_rpm = float(entry_rpm.get())
        accel_rps = float(entry_accel.get())
        decel_rps = float(entry_decel.get())
        
        # 1. 모터를 Operation Enable로 재활성화
        send_sdo_write(0x6040, 0x00, [0x06, 0x00], 2)
        send_sdo_write(0x6040, 0x00, [0x07, 0x00], 2)
        send_sdo_write(0x6040, 0x00, [0x0F, 0x00], 2)
        time.sleep(0.5) 
        
        # 2. 속도, 가속도, 감속도 데이터 계산
        velocity_data = calculate_velocity_data(target_rpm, ENCODER_CPR)
        accel_data = calculate_accel_data(accel_rps, ENCODER_CPR)
        decel_data = calculate_accel_data(decel_rps, ENCODER_CPR)
        
        # 3. 가속/감속 및 속도 설정
        send_sdo_write(0x6083, 0x00, accel_data, 4); time.sleep(0.05)
        send_sdo_write(0x6084, 0x00, decel_data, 4); time.sleep(0.05)
        send_sdo_write(0x60FF, 0x00, velocity_data, 4) # 부호가 포함된 속도 데이터 전송
        
        direction = "정방향" if target_rpm >= 0 else "역방향"
        messagebox.showinfo("구동 명령", f"모터 구동 시작: {target_rpm} RPM ({direction}), 가속: {accel_rps} RPM/s")

    except ValueError:
        messagebox.showerror("입력 오류", "모든 입력 값은 숫자여야 합니다.")
    except Exception as e:
        messagebox.showerror("구동 오류", f"모터 구동 명령 중 오류 발생: {e}")


def stop_motor():
    """Target Velocity 0을 사용하여 모터를 감속 정지시키고 비활성화합니다."""
    if not bus:
        messagebox.showerror("CAN 오류", "먼저 'CAN 초기화' 버튼을 눌러 버스를 연결하세요.")
        return

    try:
        # 입력 값 읽기
        target_rpm = float(entry_rpm.get())
        decel_rps = float(entry_decel.get())
        
        # 감속 시간 계산 시 속도의 '절대값' 사용
        target_rpm_abs = abs(target_rpm)
        
        if target_rpm_abs <= 0: target_rpm_abs = 1.0
        if decel_rps <= 0: decel_rps = 1.0
             
        decel_time = target_rpm_abs / decel_rps # 시간은 항상 양수
        
        # 1. Target Velocity를 0으로 설정하여 감속 시작
        zero_data = [0x00, 0x00, 0x00, 0x00]
        send_sdo_write(0x60FF, 0x00, zero_data, 4)
        
        messagebox.showinfo("정지 명령", f"감속 시작. 예상 감속 시간: {decel_time:.1f}초")
        
        # 2. 감속 시간만큼 대기
        time.sleep(decel_time + 0.5) 
        
        # 3. Control word = 06h (최종 비활성화)
        send_sdo_write(0x6040, 0x00, [0x06, 0x00], 2)
        messagebox.showinfo("정지 완료", "모터 최종 정지 및 비활성화 완료.")

    except ValueError:
        messagebox.showerror("입력 오류", "RPM/감속 RPM 입력 값에 문제가 있습니다.")
    except Exception as e:
        messagebox.showerror("정지 오류", f"모터 정지 명령 중 오류 발생: {e}")


def on_closing():
    """GUI 종료 시 CAN 버스를 정리하고 스레드를 종료합니다."""
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


# --- 5. Tkinter GUI 레이아웃 (이전과 동일) ---

root = tk.Tk()
root.title("ELD2-CAN 모터 제어 GUI")
root.geometry("400x350")

# CAN 초기화 버튼
btn_init = tk.Button(root, text="CAN 초기화 및 모터 준비", command=initialize_can_bus, bg='lightblue')
btn_init.pack(pady=10, padx=20, fill='x')

# 입력 프레임
input_frame = tk.Frame(root)
input_frame.pack(pady=10, padx=20)

# 텍스트박스 1: RPM (음수 입력 가능)
tk.Label(input_frame, text="1. 목표 RPM (예: -200)").grid(row=0, column=0, padx=5, pady=5, sticky='w')
entry_rpm = tk.Entry(input_frame)
entry_rpm.insert(0, "200")
entry_rpm.grid(row=0, column=1, padx=5, pady=5)

# 텍스트박스 2: 가속 RPM/s
tk.Label(input_frame, text="2. 가속 RPM/s (예: 50)").grid(row=1, column=0, padx=5, pady=5, sticky='w')
entry_accel = tk.Entry(input_frame)
entry_accel.insert(0, "50.0")
entry_accel.grid(row=1, column=1, padx=5, pady=5)

# 텍스트박스 3: 감속 RPM/s
tk.Label(input_frame, text="3. 감속 RPM/s (예: 50)").grid(row=2, column=0, padx=5, pady=5, sticky='w')
entry_decel = tk.Entry(input_frame)
entry_decel.insert(0, "50.0")
entry_decel.grid(row=2, column=1, padx=5, pady=5)

# 버튼 프레임
button_frame = tk.Frame(root)
button_frame.pack(pady=10, padx=20, fill='x')

# 버튼 1: 모터 동작
btn_run = tk.Button(button_frame, text="▶ 모터 구동", command=run_motor, bg='lightgreen')
btn_run.pack(side='left', expand=True, fill='x', padx=5)

# 버튼 2: 모터 정지
btn_stop = tk.Button(button_frame, text="■ 모터 정지", command=stop_motor, bg='salmon')
btn_stop.pack(side='right', expand=True, fill='x', padx=5)

# 창 닫기 시 이벤트 처리
root.protocol("WM_DELETE_WINDOW", on_closing)

# GUI 실행
root.mainloop()