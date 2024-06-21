# ble uart with AES-CCM Sample

# 준비사항
1. softdevice 17.1.0 다운로드
2. ses 5.42a 다운로드
3. gcc 다운로드 : https://developer.arm.com/downloads/-/gnu-rm/9-2020-q2-update
4. external/micro-ecc의 build_all.bat 스크립트를 실행해야 라이브러리가 생성되며, 해당 앱에서 이 라이브러리를 참조함
5. example의 aes ccm 참고


# 사용법
nrf connect앱으로 단말기와 연결한 후 "1" 메시지를 보내면, 단말기 내에서 암호화 및 복호화 실행