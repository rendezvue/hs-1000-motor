#!/bin/bash

# ----------------------------------------------------------------------
# ELD2-CAN 모터 구동 풀 시퀀스 (Node 5, 200 RPM)
# - 속도 값: 33334 (200 RPM 가정)으로 수정
# ----------------------------------------------------------------------

CAN_IF="can0"
NMT_ID="000"          
SDO_TX_ID="605"       
# 200 RPM Target Value (33334 counts/s). Little Endian: 36 82 00 00
VELOCITY_ACCEL_DATA="36820000" 

echo "--- 1. CANopen NMT 초기화 및 필수 조건 설정 ---"

# 1. Reset all nodes. (81 00)
echo "1. NMT Reset All Nodes (0x81) 명령 전송..."
cansend ${CAN_IF} ${NMT_ID}#8100
sleep 2.0 

# 1.1. Heartbeat Producer Time (1017h:00 = 1000ms) 설정
echo "1.1. Heartbeat Producer Time (1000ms) 설정 시도..."
cansend ${CAN_IF} ${SDO_TX_ID}#2B171000E8030000 
sleep 0.2

# ----------------------------------------------------------------------
# 2. PDO Mapping 설정 (Operational 진입 조건)
# - 100 RPM 시퀀스에서 설정이 완료되었다면 이 블록은 ACK만 받을 것입니다.
# ----------------------------------------------------------------------

echo "--- 2. RPDO1 Mapping 설정 (6040h, 60FFh) ---"
# RPDO1 Mapping 개수 0으로 설정 (수정 전 잠금 해제)
cansend ${CAN_IF} ${SDO_TX_ID}#2F00160000000000
# 1st object: 6040h:00 (Controlword, 16 bits)
cansend ${CAN_IF} ${SDO_TX_ID}#230116001000406000
# 2nd object: 60FFh:00 (Target Velocity, 32 bits)
cansend ${CAN_IF} ${SDO_TX_ID}#230216002000FF6000
# RPDO1 Mapping 개수 2로 설정 (매핑 확정)
cansend ${CAN_IF} ${SDO_TX_ID}#2F0016000200000000
sleep 0.1

echo "--- 3. RPDO1 통신 파라미터 설정 및 NMT Start ---"
# RPDO1 통신 ID 설정 (0x205)
cansend ${CAN_IF} ${SDO_TX_ID}#230114000502000000
# RPDO1 전송 Type 설정 (FFh: 비동기/이벤트 기반)
cansend ${CAN_IF} ${SDO_TX_ID}#2F021400FF000000
sleep 0.1

# 3.3. NMT Start Node 5 (0x01 05) 명령 전송
echo "3.3. NMT Start Node 5 (0x01 05) 명령 전송... (Operational 진입 시도)"
cansend ${CAN_IF} ${NMT_ID}#0105000000000000
sleep 1.0 

# ----------------------------------------------------------------------
# 4. DSP402 상태 전환 시퀀스 (SDO Write)
# ----------------------------------------------------------------------
echo "--- 4. DSP402 상태 전환 시작 ---"

# Fault Reset (80h) 및 클리어 (00h)
cansend ${CAN_IF} ${SDO_TX_ID}#2B40600080000000
cansend ${CAN_IF} ${SDO_TX_ID}#2B40600000000000
sleep 0.1 

# 3. Shutdown (06h)
cansend ${CAN_IF} ${SDO_TX_ID}#2B40600006000000
sleep 0.1

# 4. Switch On (07h)
cansend ${CAN_IF} ${SDO_TX_ID}#2B40600007000000
sleep 0.1

# 5. Operation Enable (0Fh)
echo "4.3. Control Word 0Fh (Operation Enable) 명령 전송..."
cansend ${CAN_IF} ${SDO_TX_ID}#2B4060000F000000
sleep 0.5

echo "--- 5. 운전 모드 및 200 RPM 속도/가속 설정 (SDO Write) ---"

# 6. Operation Mode = 3h
cansend ${CAN_IF} ${SDO_TX_ID}#2F60600003000000
sleep 0.1

# 7. Profile Acceleration (33334 counts/s)
cansend ${CAN_IF} ${SDO_TX_ID}#23836000${VELOCITY_ACCEL_DATA}
sleep 0.1

# 8. Profile Velocity (33334 counts/s, 모터 구동 시작)
echo "5.3. 목표 속도 설정 및 모터 구동 시작 (200 RPM 예상)."
cansend ${CAN_IF} ${SDO_TX_ID}#23FF6000${VELOCITY_ACCEL_DATA}
sleep 3

echo "--- 6. 모터 정지 및 비활성화 ---"

# 9. Control word = 07h (정지)
cansend ${CAN_IF} ${SDO_TX_ID}#2B40600007000000
sleep 0.1

# 10. Control word = 06h (최종 비활성화)
cansend ${CAN_IF} ${SDO_TX_ID}#2B40600006000000

echo "--- 전체 시퀀스 완료 ---"