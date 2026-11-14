# ESP32 Touch Angle Finder — Axicube Replica

Waveshare **ESP32-S3-Touch-LCD-1.69**용 터치 각도계 펌웨어입니다.
QMI8658 가속도계와 LVGL 화면을 사용해 상대각·절대각을 표시합니다.

현재 소스는 **0.0.2 이후 개발 버전**으로, 1D/2D 모드를 포함합니다.
화면 버전 문자열은 아직 `v0.0.2`이므로 기존 0.0.2 배포 ZIP과 현재 소스를 구분해야 합니다.
변경 이력은 [CHANGELOG](CHANGELOG.md)를 참고하세요.

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
