sudo modprobe can
sudo modprobe can_raw
sudo modprobe slcan
sudo modprobe can_dev

ls /dev/ttyACM*
# 예시: /dev/ttyACM0 만 보인다면 이 포트가 CANable입니다.
sudo slcand -o -c -s6 /dev/ttyACM3 can0
# 가상 CAN 인터페이스 띄우기
sudo ip link set can0 type can bitrate 500000
# can0 비트레이트 설정
sudo ip link set can0 up
# can0 up
