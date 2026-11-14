# 외부 구성요소와 자료 출처

프로젝트 전체를 포괄하는 라이선스는 아직 지정하지 않았습니다.
아래 구성요소와 참고 자료에는 각 원저작자의 고지가 별도로 적용됩니다.

| 구성요소 | 사용 위치 / 출처 |
| --- | --- |
| Waveshare ESP32-S3-Touch-LCD-1.69 예제·핀 정보 | [공식 자료](https://www.waveshare.com/wiki/ESP32-S3-Touch-LCD-1.69), `pin_config.h` |
| Roboto Mono Light | `roboto_mono_48.c`의 숫자·기호 서브셋, [원본](https://github.com/googlefonts/RobotoMono), [OFL 1.1](licenses/RobotoMono-OFL.txt) |
| LVGL | 외부 설치, [lvgl/lvgl](https://github.com/lvgl/lvgl) |
| Arduino_GFX | 외부 설치, [Arduino_GFX](https://github.com/moononournation/Arduino_GFX) |
| Arduino_DriveBus | 외부 설치, [Arduino_DriveBus](https://github.com/Xk-w/Arduino_DriveBus) |
| SensorLib | 외부 설치, [SensorLib](https://github.com/lewisxhe/SensorLib) |
| ESP32 Arduino core / esptool | 외부 설치 또는 배포 도구 다운로드, [Espressif](https://github.com/espressif) |

`docs/hardware/`의 Waveshare 회로도, ETA6098 및 QMI8658C 데이터시트는
원 제조사 자료입니다. 프로젝트 소스의 라이선스로 재지정하지 않습니다.
`docs/measurements/`는 기존 측정 시트 원본을 보관합니다.

네이티브 업데이터 패키지는 esptool 원본 출처·체크섬과 소스 배포본을 기록하는
별도 구조를 사용합니다. 자세한 내용은 [배포 안내](tools/NATIVE_RELEASE.md)를 참고합니다.
이 목록은 모든 의존성의 라이선스 파일을 대신하지 않습니다.
