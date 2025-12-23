import can
import time
import struct
import tkinter as tk
from tkinter import scrolledtext, ttk
import threading
import queue

# --- 1. 디자인 및 설정 상수 ---
COLOR_BG_DARK = '#1E1E1E'
COLOR_TEXT_RPM = '#00FF00'
COLOR_TEXT_TEMP = '#FFA500' 
COLOR_BTN_INIT = '#D1E8FF'
COLOR_BTN_RUN = '#C1FFC1'
COLOR_BTN_STOP = '#FFC1C1'
FONT_DASHBOARD = ('Consolas', 22, 'bold')
FONT_LOG = ('Consolas', 9)

CAN_INTERFACE = 'can0'
NODE_ID = 5
ENCODER_CPR = 10000
SDO_TX_ID = 0x600 + NODE_ID
NMT_ID = 0x000

# 글로벌 제어 변수
bus = None
bus_lock = threading.Lock()
log_queue = queue.Queue()
current_task_id = 0  # 작업 충돌 방지를 위한 ID
motor_state = "IDLE"
state_lock = threading.Lock()

# 모니터링 데이터
actual_rpm = 0.0
actual_torque_pct = 0.0
actual_overload_val = 0.0
actual_temp = 0.0 

# --- 2. 통신 함수 ---
def log(message):
    log_queue.put(f"[{time.strftime('%H:%M:%S')}] {message}")

def send_can_raw(arbitration_id, data):
    if not bus: return
    msg = can.Message(arbitration_id=arbitration_id, data=data + [0]*(8-len(data)), is_extended_id=False)
    with bus_lock:
        try: bus.send(msg)
        except Exception as e: log(f"❌ CAN Error: {e}")

def send_sdo_write(index, subindex, data, data_len):
    csid = {1: 0x2F, 2: 0x2B, 4: 0x23}.get(data_len)
    payload = [csid, index & 0xFF, (index >> 8) & 0xFF, subindex] + data
    send_can_raw(SDO_TX_ID, payload)

# --- 3. 모니터링 루틴 (매뉴얼 주소 반영) ---
def monitor_motor_status():
    global actual_rpm, actual_torque_pct, actual_overload_val, actual_temp
    if not bus: return
    
    with bus_lock:
        try:
            # 1. 실제 속도 (0x606C)
            bus.send(can.Message(arbitration_id=SDO_TX_ID, data=[0x40, 0x6C, 0x60, 0x00, 0, 0, 0, 0], is_extended_id=False))
            reply = bus.recv(timeout=0.005)
            if reply and reply.data[1:3] == b'\x6C\x60':
                raw_vel = struct.unpack('<i', reply.data[4:8])[0]
                actual_rpm = (raw_vel * 60.0) / ENCODER_CPR

            # 2. 실제 토크 (0x6077) - 0.1% 단위
            bus.send(can.Message(arbitration_id=SDO_TX_ID, data=[0x40, 0x77, 0x60, 0x00, 0, 0, 0, 0], is_extended_id=False))
            reply = bus.recv(timeout=0.005)
            if reply and reply.data[1:3] == b'\x77\x60':
                actual_torque_pct = abs(struct.unpack('<h', reply.data[4:6])[0] / 10.0)

            # 3. 드라이버 온도 (매뉴얼 Index 0x2033)
            bus.send(can.Message(arbitration_id=SDO_TX_ID, data=[0x40, 0x33, 0x20, 0x00, 0, 0, 0, 0], is_extended_id=False))
            reply = bus.recv(timeout=0.005)
            if reply and reply.data[1:3] == b'\x33\x20':
                actual_temp = float(struct.unpack('<h', reply.data[4:6])[0])

            # 4. 오버로드 부하율 (매뉴얼 Index 0x200F)
            bus.send(can.Message(arbitration_id=SDO_TX_ID, data=[0x40, 0x0F, 0x20, 0x00, 0, 0, 0, 0], is_extended_id=False))
            reply = bus.recv(timeout=0.005)
            if reply and reply.data[1:3] == b'\x0F\x20':
                 actual_overload_val = float(struct.unpack('<h', reply.data[4:6])[0])
        except: pass

def monitor_loop():
    while True:
        if bus and motor_state != "INITIALIZING": monitor_motor_status()
        time.sleep(0.001)

# --- 4. 제어 로직 (중단 및 재개 기능 포함) ---
def task_initialize():
    global bus, motor_state
    try:
        with state_lock: motor_state = "INITIALIZING"
        log("🔄 초기화 시작...")
        with bus_lock:
            if bus: bus.shutdown()
            bus = can.interface.Bus(channel=CAN_INTERFACE, interface='socketcan')
        
        send_can_raw(NMT_ID, [0x81, 0x00]); time.sleep(2.0) # Reset
        send_sdo_write(0x1017, 0x00, [0xE8, 0x03], 2); time.sleep(0.1)
        send_can_raw(NMT_ID, [0x01, NODE_ID]); time.sleep(0.5) # Start
        
        send_sdo_write(0x6060, 0x00, [0x03], 1)       # Mode: Velocity
        send_sdo_write(0x6040, 0x00, [0x06, 0x00], 2) # Shutdown
        log("✅ 준비 완료")
        with state_lock: motor_state = "IDLE"
    except Exception as e: log(f"❌ 초기화 실패: {e}")

def task_run_motor(rpm, accel, decel):
    global current_task_id, motor_state
    if not bus: return
    with state_lock:
        current_task_id += 1 # 작업 ID 증가 (이전 정지 시퀀스 무력화)
        motor_state = "RUNNING"
    
    # 구동 시퀀스 (이미 돌고 있어도 덮어쓰기)
    send_sdo_write(0x6040, 0x00, [0x0F, 0x00], 2)
    accel_val = int(accel * (ENCODER_CPR / 60.0))
    decel_val = int(decel * (ENCODER_CPR / 60.0))
    vel_val = int((rpm * ENCODER_CPR) / 60)
    
    send_sdo_write(0x6083, 0x00, list(struct.pack('<I', accel_val)), 4)
    send_sdo_write(0x6084, 0x00, list(struct.pack('<I', decel_val)), 4)
    send_sdo_write(0x60FF, 0x00, list(struct.pack('<i', vel_val)), 4)
    log(f"▶ 구동 시작: {rpm} RPM")

def task_stop_motor(rpm, decel):
    global current_task_id, motor_state
    if not bus: return
    
    # [개선] 이미 정지 중이거나 정지 상태라면 명령 무시
    with state_lock:
        if motor_state == "STOPPING":
            log("⚠️ 이미 감속 정지 중입니다.")
            return
        if motor_state == "IDLE":
            log("ℹ️ 모터가 이미 정지 상태입니다.")
            return
            
        current_task_id += 1
        this_id = current_task_id
        motor_state = "STOPPING"
    
    log(f"■ 감속 정지 시작 (ID: {this_id})")
    send_sdo_write(0x60FF, 0x00, [0,0,0,0], 4) # 속도 0 명령
    
    # 감속 대기 루프 (재구동 시 this_id != current_task_id 조건에 의해 파기됨)
    decel_time = abs(rpm / decel) if decel > 0 else 1
    steps = int(decel_time * 20) + 10
    for _ in range(steps):
        # 재구동 버튼이 눌려 ID가 바뀌면 이 정지 루프를 즉시 탈출
        if this_id != current_task_id: 
            return 
        time.sleep(0.05)
    
    # 루프를 정상 종료했다면(중간에 가로채기 없었다면) 비활성화
    if this_id == current_task_id:
        send_sdo_write(0x6040, 0x00, [0x06, 0x00], 2) # Disable Operation
        with state_lock: motor_state = "IDLE"
        log("✅ 완전 정지 완료")
# --- 5. GUI 구성 ---
root = tk.Tk()
root.title("ELD2-CAN70 Monitoring System")
root.geometry("480x820")

db_frame = tk.Frame(root, bg=COLOR_BG_DARK, pady=15)
db_frame.pack(fill='x', padx=20, pady=10)

rpm_container = tk.Frame(db_frame, bg=COLOR_BG_DARK)
rpm_container.pack(side='left', expand=True)
tk.Label(rpm_container, text="VELOCITY", bg=COLOR_BG_DARK, fg='white', font=('Arial', 9)).pack()
lbl_rpm = tk.Label(rpm_container, text="0.0 RPM", bg=COLOR_BG_DARK, fg=COLOR_TEXT_RPM, font=FONT_DASHBOARD)
lbl_rpm.pack()

temp_container = tk.Frame(db_frame, bg=COLOR_BG_DARK)
temp_container.pack(side='right', expand=True)
tk.Label(temp_container, text="DRIVE TEMP (0x2033)", bg=COLOR_BG_DARK, fg='white', font=('Arial', 9)).pack()
lbl_temp = tk.Label(temp_container, text="0°C", bg=COLOR_BG_DARK, fg=COLOR_TEXT_TEMP, font=FONT_DASHBOARD)
lbl_temp.pack()

torque_frame = tk.Frame(root, padx=20)
torque_frame.pack(fill='x', pady=5)
tk.Label(torque_frame, text="Torque (%)", font=('Arial', 9, 'bold')).pack(side='left')
lbl_torque_text = tk.Label(torque_frame, text="0.0 %")
lbl_torque_text.pack(side='right')
torque_bar = ttk.Progressbar(root, orient='horizontal', length=400, mode='determinate')
torque_bar.pack(padx=20, pady=5, fill='x')

overload_frame = tk.Frame(root, padx=20)
overload_frame.pack(fill='x', pady=5)
tk.Label(overload_frame, text="Over-load Ratio (0x200F)", font=('Arial', 9, 'bold'), fg='red').pack(side='left')
lbl_overload_text = tk.Label(overload_frame, text="0.0 %", font=('Arial', 9, 'bold'), fg='red')
lbl_overload_text.pack(side='right')

def update_gui():
    lbl_rpm.config(text=f"{actual_rpm:.1f} RPM")
    lbl_temp.config(text=f"{actual_temp:.0f}°C")
    lbl_torque_text.config(text=f"{actual_torque_pct:.1f} %")
    torque_bar['value'] = actual_torque_pct
    lbl_overload_text.config(text=f"{actual_overload_val:.1f} %")
    while not log_queue.empty():
        msg = log_queue.get()
        log_area.configure(state='normal')
        log_area.insert(tk.END, msg + "\n")
        log_area.see(tk.END)
        log_area.configure(state='disabled')
    root.after(50, update_gui)

btn_init = tk.Button(root, text="CAN 통신 초기화", command=lambda: threading.Thread(target=task_initialize, daemon=True).start(), bg=COLOR_BTN_INIT, height=2)
btn_init.pack(pady=5, fill='x', padx=20)

input_frame = tk.LabelFrame(root, text="제어 설정")
input_frame.pack(pady=5, padx=20, fill='x')
entry_rpm = tk.Entry(input_frame); entry_rpm.insert(0, "1000"); entry_rpm.grid(row=0, column=1)
tk.Label(input_frame, text="RPM").grid(row=0, column=0)
entry_accel = tk.Entry(input_frame); entry_accel.insert(0, "100"); entry_accel.grid(row=1, column=1)
tk.Label(input_frame, text="가속").grid(row=1, column=0)
entry_decel = tk.Entry(input_frame); entry_decel.insert(0, "100"); entry_decel.grid(row=2, column=1)
tk.Label(input_frame, text="감속").grid(row=2, column=0)

tk.Button(root, text="▶ 모터 구동", command=lambda: threading.Thread(target=task_run_motor, args=(float(entry_rpm.get()), float(entry_accel.get()), float(entry_decel.get())), daemon=True).start(), bg=COLOR_BTN_RUN, height=2, font=('Arial', 10, 'bold')).pack(pady=5, fill='x', padx=20)
tk.Button(root, text="■ 감속 정지", command=lambda: threading.Thread(target=task_stop_motor, args=(float(entry_rpm.get()), float(entry_decel.get())), daemon=True).start(), bg=COLOR_BTN_STOP, height=2, font=('Arial', 10, 'bold')).pack(pady=5, fill='x', padx=20)

log_area = scrolledtext.ScrolledText(root, height=10, state='disabled', bg=COLOR_BG_DARK, fg='white', font=FONT_LOG)
log_area.pack(pady=10, padx=20, fill='both', expand=True)

threading.Thread(target=monitor_loop, daemon=True).start()
update_gui()
root.mainloop()