import time
import can
import struct
import random

def send_test():
    print("라즈베리파이 CAN 전송 시작! (Ctrl+C로 종료)")

    # [수정 1] 경고 메시지 해결 ('bustype' -> 'interface')
    try:
        bus = can.interface.Bus(channel='can0', interface='socketcan')
    except OSError:
        print("오류: CAN 장치를 찾을 수 없습니다.")
        print("   -> 터미널에 'sudo ip link set can0 up ...' 명령어를 먼저 입력하세요.")
        return

    try:
        while True:
            # 1. 테스트용 랜덤 값 생성 (-1.0 ~ 1.0)
            throttle = round(random.uniform(-1.0, 1.0), 2)
            steer = round(random.uniform(-1.0, 1.0), 2)

            # 2. 데이터 패킹 (STM32가 받을 포맷: Big-endian short 2개)
            t_int = int(throttle * 1000)
            s_int = int(steer * 1000)
            data = struct.pack('>hh', t_int, s_int)

            # 3. 메시지 생성 (ID: 0x100)
            msg = can.Message(arbitration_id=0x100, data=data, is_extended_id=False)

            try:
                # [수정 2] 전송 시도 (에러 처리 추가)
                bus.send(msg)
                print(f"[TX] Throttle: {throttle}, Steer: {steer}")
                
            except can.CanError as e:
                # STM32가 꺼져있거나 선이 빠져서 버퍼가 꽉 차도 프로그램이 죽지 않음
                print(f"전송 실패 (STM32 응답 없음 / 선 확인 필요): {e}")
                # 버퍼가 좀 비워질 때까지 더 오래 대기
                time.sleep(1.0)

            # 0.5초 대기
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n프로그램 종료.")

if __name__ == "__main__":
    send_test()
