# 빌드 환경

모든 명령은 저장소 루트(`LVGL_Axicube_Replica/`)에서 실행합니다.

## 대상과 의존성

| 구성요소 | 기존 검증 버전 | 설치 경로 |
| --- | --- | --- |
| Arduino CLI | 1.5.1 | CLI 또는 Arduino IDE 2에 포함된 실행 파일 |
| ESP32 Arduino core | 3.3.11 | Boards Manager / `core install` |
| GFX Library for Arduino | 1.6.7 | Library Manager |
| lvgl | 9.5.0 | Library Manager |
| SensorLib | 0.4.1 | Library Manager |
| Driver Bus Library Based on Arduino | 1.0.1 | Waveshare 예제의 `Arduino_DriveBus` 폴더 |

`Arduino_DriveBus`는 Arduino 라이브러리 인덱스에서 제공되지 않는 의존성입니다.
[Waveshare 보드 자료](https://www.waveshare.com/wiki/ESP32-S3-Touch-LCD-1.69)의
Demo에서 `Arduino/libraries/Arduino_DriveBus`를 Arduino 스케치북의 `libraries/`에 설치합니다.
`library.properties`의 버전 `1.0.1`과 `Arduino_DriveBus_Library.h`의 존재를 확인합니다.
이 저장소에 라이브러리 전체를 복사하거나 라이브러리 내부 파일을 변경할 필요는 없습니다.

핀 정의는 저장소의 `pin_config.h`를 사용합니다. 기존 Waveshare `Mylibrary` 설치는 필요 없습니다.
LVGL은 저장소의 `lv_conf.h`를 사용하며, 현재 검증 대상은 LVGL 9.5.0입니다.
소스에 LVGL 8 호환 분기도 있지만 해당 조합 전체를 검증한 것은 아닙니다.

## Arduino CLI

`arduino-cli`를 PATH에 추가합니다. macOS에서 IDE에 포함된 실행 파일을 직접 쓸 때는
`"/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli"`로 대체합니다.

```sh
arduino-cli core update-index \
  --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core install esp32:esp32@3.3.11 \
  --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli lib update-index
arduino-cli lib install "GFX Library for Arduino@1.6.7" "lvgl@9.5.0" "SensorLib@0.4.1"
```

별도로 위 `Arduino_DriveBus` 설치를 마친 후 컴파일합니다.

```sh
arduino-cli compile \
  --fqbn esp32:esp32:waveshare_esp32_s3_touch_lcd_169:PartitionScheme=app3M_fat9M_16MB,PSRAM=disabled \
  --build-path build .
```

보드는 **Waveshare ESP32-S3-Touch-LCD-1.69**, 파티션은 `app3M_fat9M_16MB`,
PSRAM은 **disabled**입니다. 다른 설정은 해당 보드 정의의 기본값을 사용합니다.
Arduino IDE에서는 같은 보드·파티션·PSRAM을 선택한 뒤 `.ino` 파일을 엽니다.

소스 폴더 이름은 반드시 `LVGL_Axicube_Replica`여야 합니다.
빌드 폴더는 Git에서 제외됩니다. 업로드는 [업데이터](../tools/README.md)로 별도 실행합니다.

## 진단 및 검증

반복 USB 진단은 기본 OFF입니다. 진단 빌드 시 compile 명령에
`--build-property compiler.cpp.extra_flags=-DAXICUBE_REL_DIAGNOSTICS=1`을 추가합니다.
115200 baud로 `REL_CONTACT`를 최대 초당 한 번 출력합니다.

```sh
python3 simulation/capture_serial.py --port /dev/cu.usbmodemYOUR_PORT --duration 60
bash tests/run_host_tests.sh
python3 -m unittest discover -s tests -p 'test_*.py' -v
```

초기 시리얼 버퍼의 과거 데이터는 분석에서 제외합니다. 하드웨어 확인 항목은
화면·터치·영점, 재부팅 후 CAL 로드, 장착면별 수평/45° 왕복과 좌우 회전 반복입니다.
과거 실기 기록은 [history](history/)에 보관합니다.
