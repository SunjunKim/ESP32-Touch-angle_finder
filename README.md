# Axicube Replica 0.0.2 — ESP32-S3 Touch LCD 1.69

Waveshare ESP32-S3-Touch-LCD-1.69용 각도계 펌웨어. 현재 REL은 가속도계만
사용하며, 자이로 적분·정지 대기·추적 손실 상태를 사용하지 않는다.
제품 코드 정리 기준일: 2026-09-08. 정밀도 인증 또는 실기 검증 완료 선언은 아니다.

## REL 사용법

1. 칼면과 스톤면 중 더 수평에 가까운 기준면에 기기를 밀착한다.
2. 기존 영점 버튼을 3초 누르고, 안내에 따라 손을 뗀다.
3. 영점 순간 중력 성분이 가장 큰 X/Y/Z축을 접촉면 법선으로 고정한다.
   `N:X/Y/Z locked`를 확인한다. 측정 중에는 자동으로 축을 바꾸지 않는다.
4. 같은 기기 접촉면으로 대상면에 부착하여 상대각을 읽는다.
   기준면 자체를 움직였거나 기기의 접촉면을 바꿨다면 다시 영점을 잡는다.

표시는 0–90°의 무부호 상대각이며 0.1° 단위다. 이동 중에도 계속 표시한다.
빨강은 움직임/필터 안정화 힌트, 하양은 안정화 힌트일 뿐 정확도 보증이 아니다.
유효한 센서 데이터가 500ms 이상 없으면 `--`로 표시한다. 출력 필터 시정수는
0.15초다. 상세 수식과 이력은 [ACCEL_REL.md](ACCEL_REL.md)를 참조한다.

완전 수평 영점 및 정렬된 단축 회전에서는 이상적인 기하값을 얻는다.
기울어진 기준면에서 상대 방위가 바뀌면 가속도계만으로 복원할 수 없는
각도 오차 및 부호 분기 경계가 남는다. 움직임 중 병진 가속도, 센서 편향,
케이스와 연마면의 비평행도도 오차 원인이다. 표시 분해능은 측정 정확도가 아니다.

## 캘리브레이션

기존 6방향 CAL 순서와 계산은 유지한다. 가능한 수평이고 평탄한 면을 사용한다.
REL 영점과 센서 캘리브레이션은 별개다. 기존 NVS 키와 진단 펌웨어의
`data_v3` 레코드 읽기/쓰기 호환성을 유지한다. CAL 진입 후 REL 영점은 다시 잡는다.
이번 5개 면 진단의 추정 보정값은 적용하지 않았다. USB 쪽 한 면이 빠진
진단을 완전한 6면 캘리브레이션으로 간주하지 않는다.

## 빌드

Arduino CLI와 ESP32 보드 패키지, Arduino_GFX_Library, lvgl, SensorLib 및
프로젝트가 사용하는 주변장치 의존성이 필요하다. 로컬 `lv_conf.h`를 사용한다.
라이브러리 내부 파일은 변경하지 않는다. LVGL 8/9 분기를 제공하지만 이번
검증은 설치된 라이브러리 조합에 한정된다.

```sh
arduino-cli compile \
  --fqbn esp32:esp32:waveshare_esp32_s3_touch_lcd_169:PartitionScheme=app3M_fat9M_16MB,PSRAM=disabled \
  --build-path /private/tmp/axicube-production .
bash tests/run_host_tests.sh
```

반복 USB 진단은 기본 OFF다. 필요할 때 컴파일 옵션
`--build-property build.extra_flags=-DAXICUBE_REL_DIAGNOSTICS=1`을 추가한다.
진단 빌드는 115200 baud로 `REL_CONTACT`를 최대 초당 한 번 비차단 출력한다.
`python3 simulation/capture_serial.py --port /dev/cu.usbmodem… --duration 60`으로
수집한다. 기존 로그처럼 초기 버퍼 데이터가 포함될 수 있으므로 분석에서 제외한다.
업로드는 별도로 수행하며 보정값 초기화/전체 플래시 삭제는 하지 않는다.

## 검증 및 개발 기록

- 활성 REL 회귀: `tests/contact_rel_test.cpp`의 중앙 23,808 자세와
  18,360 장착면 등가성 표본. 후자는 임의 방위에서 실제 각도 정확도 검증이 아니다.
- 실측 45° 지그 표시: 두 장착 조건에서 46.7° / 45.8°; 복귀 0.0–0.1°.
  지그 자체 정밀도는 확인하지 않았으므로 제품 정확도 수치로 사용하지 않는다.
- [5개 면 진단](simulation/hardware_logs/FIVE_FACE_CONTACT_DIAGNOSTIC_20260907.md):
  방향 의존 편차, 접촉 상태 및 잔여 편향 가능성. 보정값 미적용.
- `simulation/`과 그 안의 `experimental/`은 개발 기록과 폐기 후보를 보관한다.
  자이로 추적기는 제품 펌웨어에 포함되지 않는다.
- `STATIONARY_REL.md`, `IMU_TUNING.md`, `HARDWARE_TEST.md`,
  `SIMULATION_FINDINGS.md`는 과거 조건에 대한 기록이다. 현재 사용법은 본 문서가 우선한다.

남은 실기 확인: production 빌드의 화면·터치·영점, 재부팅 후 기존 CAL 로드,
장착면별 수평/45° 왕복과 좌우 회전 반복. 이 정리 작업에서는 재업로드하지 않는다.
