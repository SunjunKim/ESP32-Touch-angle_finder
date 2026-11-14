# ESP32 Touch Angle Finder — Axicube Replica

Waveshare **ESP32-S3-Touch-LCD-1.69**용 터치 각도계 펌웨어입니다.
QMI8658 가속도계와 LVGL 화면을 사용해 상대각·절대각을 표시합니다.

현재 배포 버전은 제품 화면에 표시되는 **v0.0.2**이며, 1D/2D 모드를 포함합니다.
아래 GitHub Release의 ZIP은 현재 소스로 만든 배포본입니다.
과거 로컬 시험용 ZIP도 같은 버전 문자열을 사용했으므로 Release에서 다시 다운로드하세요.
변경 이력은 [CHANGELOG](CHANGELOG.md)를 참고하세요.

## 기존 제품 사용자: USB로 업데이트

**Arduino IDE·Python·라이브러리를 설치할 필요가 없습니다.** ZIP에 펌웨어와
업데이트 실행 도구가 포함되어 있으며, 다운로드 후에는 인터넷 없이 동작합니다.

1. [v0.0.2 다운로드 페이지](https://github.com/SunjunKim/ESP32-Touch-angle_finder/releases/tag/v0.0.2)의
   **Assets**에서 PC에 맞는 ZIP을 받습니다. `Source code`는 개발자용입니다.

   | PC | 다운로드 |
   | --- | --- |
   | Windows 64비트 (Intel/AMD) | [axicube-0.0.2-windows-amd64.zip](https://github.com/SunjunKim/ESP32-Touch-angle_finder/releases/download/v0.0.2/axicube-0.0.2-windows-amd64.zip) |
   | macOS Apple Silicon (M 시리즈) | [axicube-0.0.2-macos-arm64.zip](https://github.com/SunjunKim/ESP32-Touch-angle_finder/releases/download/v0.0.2/axicube-0.0.2-macos-arm64.zip) |
   | macOS Intel | [axicube-0.0.2-macos-amd64.zip](https://github.com/SunjunKim/ESP32-Touch-angle_finder/releases/download/v0.0.2/axicube-0.0.2-macos-amd64.zip) |

2. ZIP을 **폴더째 압축 해제**합니다. 실행 파일만 꺼내거나 ZIP 안에서 실행하지 마세요.
3. 제품을 **데이터 전송 가능한 USB 케이블**로 연결하고 시리얼 모니터를 닫습니다.
4. 압축 해제한 폴더에서 실행합니다.
   - **Windows:** 터미널에서 `.\update.cmd` 실행.
   - **macOS:** 터미널에서 `bash update.command` 실행.
5. 표시된 포트가 해당 제품인지 확인하고 `UPDATE`를 입력합니다.
   완료될 때까지 USB를 유지하면 쓰기·검증 후 재부팅을 요청합니다.
6. 시작 화면의 `v0.0.2`, 터치와 모드 전환을 확인한 뒤 **영점을 다시 잡습니다**.

기존 Axicube와 같은 파티션 구성이면 저장된 CAL 보정값을 유지하며 전체 플래시를 지우지 않습니다.
다른 펌웨어나 파티션 구성에서의 보정값 유지는 보장하지 않습니다.

기기가 여러 개면 Windows는 `.\update.cmd -Port COM8`, macOS는
`bash update.command --port /dev/cu.usbmodem실제포트`로 선택합니다.
자동 연결에 실패하면 BOOT를 누른 채 RESET을 눌렀다 떼고 BOOT를 놓은 뒤 다시 실행합니다.
수동 다운로드 모드에서 업데이트했다면 RESET을 직접 눌러야 할 수 있습니다.

OS 보안 경고는 [상세 업데이트 안내](tools/NATIVE_RELEASE.md)를 참고하세요.
비공개 저장소의 다운로드에는 접근 권한이 있는 GitHub 계정 로그인이 필요합니다.

## 주요 기능

| 모드 | 동작 |
| --- | --- |
| 2D | 영점에서 접촉면 법선을 고정해 전체 상대 경사를 표시, 0–90° |
| 1D | ABS pitch와 같은 축의 영점 대비 각도 차이, 0–180° |
| ABS | 절대 경사와 수평계 표시 |
| CAL | 6방향 가속도계 보정 및 비휘발성 저장 |

- 시작 모드: **2D**. 모드 버튼: **2D → 1D → ABS → CAL**.
- 1D/2D는 영점을 공유합니다. 정지한 기준면에서 영점 버튼을 **1초** 누릅니다.
- 배터리 전압, 추정 잔량·외부 전원 연결 표시, 유휴 화면 절전과 전원 종료를 지원합니다.
- Windows/macOS USB 업데이트 및 사전 컴파일 ZIP 생성 도구를 제공합니다.

표시 분해능은 0.1°이며 측정 정확도를 의미하지 않습니다. 가속도계만으로 계산하므로
기울어진 기준면의 방위 변화나 이동 중 가속도에 영향을 받습니다.
사용 절차와 한계는 [사용법](docs/USAGE.md), 계산식은 [상대각 알고리즘](docs/ACCEL_REL.md)에 정리되어 있습니다.

## 시작하기

### 소스 빌드

Arduino는 스케치 폴더 이름과 `.ino` 파일 이름이 같아야 합니다. 다음 이름으로 내려받으세요.

```sh
git clone https://github.com/SunjunKim/ESP32-Touch-angle_finder.git LVGL_Axicube_Replica
cd LVGL_Axicube_Replica
```

GitHub의 Download ZIP을 사용했다면 압축 해제한 폴더를 `LVGL_Axicube_Replica`로 변경합니다.
[빌드 안내](docs/BUILD.md)에 따라 의존성을 설치한 뒤 실행합니다.

```sh
arduino-cli compile \
  --fqbn esp32:esp32:waveshare_esp32_s3_touch_lcd_169:PartitionScheme=app3M_fat9M_16MB,PSRAM=disabled \
  --build-path build .
```

핀 설정(`pin_config.h`)과 LVGL 설정(`lv_conf.h`)은 저장소에 포함되어 있습니다.
별도의 `Mylibrary`는 필요하지 않습니다.

### 장치 업데이트

소스에서 빌드·업로드하려면 [USB 업데이트 안내](tools/README.md)를 따릅니다.
배포자가 제공한 OS별 ZIP은 [사전 컴파일 업데이트 안내](tools/NATIVE_RELEASE.md)를 따릅니다.
ZIP은 개발자가 별도로 생성하는 산출물이며 소스 저장소에는 포함하지 않습니다.

### 테스트

C++17 컴파일러(기본 `clang++`), Node.js, Python 3.9 이상이 필요합니다.
하드웨어 연결이나 Arduino 라이브러리 없이 실행할 수 있습니다.

```sh
bash tests/run_host_tests.sh
python3 -m unittest discover -s tests -p 'test_*.py' -v
```

GitHub Actions에서도 같은 테스트를 실행합니다. 호스트 테스트는 계산·UI 상태 전이·업데이터를
검증하며, 실제 장치의 정확도·터치·전원 동작은 별도 확인이 필요합니다.

## 저장소 구성

| 경로 | 내용 |
| --- | --- |
| `LVGL_Axicube_Replica.ino` | 장치 초기화, IMU 태스크, 화면·전원 관리 |
| `src/` | 화면 페이지, 상대각 계산, 보정 저장, 배터리 모델 |
| `pin_config.h`, `lv_conf.h`, `roboto_mono_48.c` | 보드 핀, LVGL 설정, 숫자 폰트 |
| `tools/` | USB 업데이트, 배포 ZIP 생성 |
| `tests/` | C++·JavaScript·Python 호스트 테스트 |
| `docs/` | 사용법, 빌드, 알고리즘, 배터리 설명, 배포 절차 |
| `docs/history/` | 과거 알고리즘·실기 검증 기록 |
| `docs/hardware/`, `docs/measurements/` | 회로도·데이터시트, 측정 원본 |
| `simulation/` | 시뮬레이션, 폐기 후보, 실측 로그 |
| `dist/`, `build/` | 생성 산출물, Git 제외 |

## 개발 자료

- [사용법·캘리브레이션](docs/USAGE.md) · [배터리 표시](docs/BATTERY.md)
- [빌드 환경](docs/BUILD.md) · [GitHub 배포 절차](docs/RELEASING.md)
- [상대각 알고리즘과 회귀 검증](docs/ACCEL_REL.md)
- [5개 면 진단 기록](simulation/hardware_logs/FIVE_FACE_CONTACT_DIAGNOSTIC_20260907.md)
- [Waveshare 공식 보드 자료](https://www.waveshare.com/wiki/ESP32-S3-Touch-LCD-1.69)

`docs/history/`와 `simulation/`에는 현재 펌웨어와 다른 실험 조건이 남아 있습니다.
현재 사용법은 본 문서와 `docs/USAGE.md`를 기준으로 합니다.

## 라이선스

별도 고지가 없는 프로젝트 코드와 문서는 **GNU General Public License v3.0 only
(GPL-3.0-only)**로 배포합니다. 전문은 [LICENSE](LICENSE)를 참고하세요.

Roboto Mono 폰트, 제조사 회로도·데이터시트 및 외부 구성요소에는 각 원저작자의
라이선스와 고지가 적용됩니다. 출처는 [THIRD_PARTY_NOTICES](THIRD_PARTY_NOTICES.md)에 기록합니다.
