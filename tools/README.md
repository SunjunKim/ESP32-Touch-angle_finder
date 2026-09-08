# USB firmware updater (Windows / macOS)

일반 사용자용: [Python 없는 사전 컴파일 배포](NATIVE_RELEASE.md)를 우선 사용한다.
개발자가 현재 소스를 먼저 컴파일한 뒤 다음 명령으로 OS별 ZIP을 생성한다.

```sh
python3 tools/package_native.py --build-dir /private/tmp/axicube-usb-debug --cache /private/tmp/axicube-native-tools
```

아래는 기존 Arduino CLI 기반 개발자용 업데이트 경로다.

Waveshare ESP32-S3-Touch-LCD-1.69용이다. 장치 탐색은 Arduino CLI,
리셋·플래시는 ESP32 core의 esptool 업로더에 맡긴다. Python 표준 라이브러리만
사용하므로 별도 pip 설치는 필요 없다. 펌웨어·라이브러리 자체는 수정하지 않는다.

## 준비

- Python 3.9 이상: macOS는 `python3`, Windows는 `py -3` 또는 `python`.
- Arduino CLI 또는 Arduino IDE 2. PATH와 일반적인 IDE 설치 위치를 탐색한다.
  찾지 못하면 `--cli "실행파일 경로"`를 지정한다.
- 검증 환경은 Arduino CLI 1.5.1 / ESP32 core 3.3.11이다.
- 소스 빌드에는 상위 [README](../README.md)의 모든 라이브러리가 필요하다.
  기존 빌드 폴더로 업데이트할 때는 소스 라이브러리가 필요 없지만,
  해당 보드 정의와 업로더가 포함된 ESP32 core는 설치되어 있어야 한다.
- 충전 전용이 아닌 USB 케이블로 연결하고 IDE 시리얼 모니터를 닫는다.

## 실행

저장소 루트에서 터미널로 실행한다. 경로에 공백이 있으면 따옴표로 감싼다.

macOS:

```sh
bash tools/update-macos.command --list
bash tools/update-macos.command
```

Windows (명령 프롬프트 또는 PowerShell):

```text
.\tools\update-windows.cmd --list
.\tools\update-windows.cmd
```

후보를 출력한 후 해당 기기가 Waveshare 1.69인지 묻는다. `UPDATE`를 입력하면
빌드부터 완료하고, 장치 일련번호를 다시 조회하여 업로드한다. 버튼을 미리 눌러
리셋할 필요는 없다. 빌드 폴더는 임시 디렉터리에 만들고 경로를 출력하며,
재사용/진단할 수 있도록 자동 삭제하지 않는다.

공통 옵션 (두 래퍼와 Python 스크립트에서 동일):

```text
--list                         탐색만 수행, 리셋/쓰기 없음
--dry-run                      대상 및 계획 확인, 빌드/리셋/쓰기 없음
--port COM8                    특정 포트 선택 (macOS는 /dev/cu.usbmodem…)
--serial DC:B4:D9:23:1A:80       USB 일련번호로 선택
--yes                          해당 후보가 올바른 모델임을 확인하고 질문 생략
--input-dir "기존 빌드 폴더"     재컴파일 없이 신뢰하는 빌드 사용
--cli "Arduino CLI 경로"        CLI 자동 검색 대신 명시
```

예: `bash tools/update-macos.command --serial DC:B4:D9:23:1A:80 --yes`.
일련번호는 이번 실기 장치의 예시다. 다른 기기는 `--list`에서 확인해야 한다.
무인 실행에는 `--serial`과 `--yes`를 함께 사용하는 것이 좋다.

`--input-dir`에는 동일 FQBN으로 만든 Arduino CLI 빌드 폴더를 지정한다.
`build.options.json` 및 `LVGL_Axicube_Replica.ino.bin`, `.bootloader.bin`,
`.partitions.bin` 파일이 필요하다. 단일 merged.bin은 받지 않는다.
보드/파티션/크기 검사는 실수 방지용이지 서명 검증이 아니다. 신뢰하는 빌드만 사용한다.

## 안전 및 리셋 동작

- USB `303A:1001`(기본 Serial/JTAG) 또는 `303A:821E`만 후보로 취급한다.
  **USB ID만으로 Waveshare 모델은 확정할 수 없다.** 다른 ESP32 모델에도 같은
  ID가 있으므로 사용자 확인이 필요하다. esptool은 칩 종류도 검사한다.
- 후보가 없거나 여러 개면 자동 선택하지 않는다. 명시 포트도 USB 필터를 통과해야 한다.
- 빌드 후 같은 USB 일련번호로 재탐색한다. 포트 번호 변경은 허용하지만
  원래 장치가 사라졌거나 일련번호가 중복되면 중단한다. 업로드 중 포트 변경/단절은
  업로더의 처리 범위를 넘으면 오류로 종료하며 다른 장치로 재시도하지 않는다.
- 설치된 ESP32 core 3.3.11의 업로드 레시피는 `--before default-reset`과
  `--after hard-reset`을 사용한다. 별도 1200-baud 터치나 선행 리셋은 하지 않는다.
- 전체 플래시 삭제를 요청하지 않는다. NVS `0x9000–0xDFFF`와 겹치지 않는
  부트로더·파티션·앱을 쓴다. 현재 프로젝트의 같은 파티션 배치에서 기존 CAL을
  유지하는 업데이트용이며, 다른 제품/파티션에서의 데이터 이행 도구는 아니다.
- 성공은 업로더의 성공 종료를 뜻한다. 화면 부팅·버전·정확도는 자동 검증하지 않는다.
  업데이트 후 화면의 `v0.0.2`, 터치와 영점, 기존 CAL 동작을 확인한다.

자동 진입 실패 시 BOOT를 누른 채 RESET을 눌렀다 떼고 BOOT를 놓은 뒤
`--list`로 다시 찾는다. 수동 다운로드 모드 진입 후에는 업로드 완료 뒤 RESET이
필요할 수 있다. USB 자체가 비활성화됐거나 케이블/드라이버가 잘못된 경우까지
소프트웨어로 자동 복구할 수는 없다. Windows에서는 장치 관리자도 확인한다.

근거: [Espressif 다운로드 모드/리셋](https://docs.espressif.com/projects/esptool/en/latest/esp32s3/esptool/entering-bootloader.html),
[USB Serial/JTAG 설명](https://docs.espressif.com/projects/esp-iot-solution/en/release-v2.0/usb/usb_overview/usb_serial_jtag.html).

## 테스트

```sh
python3 -B -m unittest discover -s tests -p 'test_update_firmware.py'
bash -n tools/update-macos.command
```

단위 테스트는 Windows/macOS 포트, 다중 후보, 장치 교체, 재열거, 빌드 검사와
dry-run의 무변경 동작을 검증한다. Windows 래퍼의 실제 실행 및 실제 플래시/리셋은
별도 실기 검증이 필요하다. 스크립트 작성 중에는 장치를 리셋하거나 플래시하지 않는다.
