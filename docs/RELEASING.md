# GitHub 업로드와 릴리스

## 소스 업로드

저장소의 `origin`은 `https://github.com/SunjunKim/ESP32-Touch-angle_finder.git`입니다.
루트 README가 시작점이며 `docs/`는 설명·참고 자료, `simulation/`은 개발 기록입니다.
`build/`, `dist/`, 캐시·OS 파일은 `.gitignore`로 제외합니다.

업로드 전 저장소 루트에서 확인합니다.

```sh
git status --short
git diff --check
bash tests/run_host_tests.sh
python3 -m unittest discover -s tests -p 'test_*.py' -v
```

변경을 검토한 뒤 커밋하고 현재 브랜치를 업로드합니다.

```sh
git add .
git diff --cached --stat
git commit -m "chore: prepare repository for GitHub"
git push -u origin HEAD
```

기존에 추적 중인 파일은 `.gitignore`만으로 제외되지 않습니다. 이 저장소의
회로도·측정 시트·실험 로그는 참고 자료로 유지합니다. 공개 범위를 바꾸려면
해당 파일과 기존 Git 이력을 별도로 검토해야 합니다.
프로젝트 전체의 라이선스는 아직 선택되지 않았으며, MIT 등의 라이선스를 임의로 적용하지 않습니다.
외부 자료의 출처는 [THIRD_PARTY_NOTICES](../THIRD_PARTY_NOTICES.md)를 참고합니다.

## 펌웨어 릴리스

현재 소스에는 0.0.2 이후 변경이 있지만 `kAppVersion`은 아직 `v0.0.2`입니다.
새 릴리스에서는 `.ino`의 버전, `CHANGELOG.md`, 배포 안내의 버전 표기를 함께 갱신합니다.
기존 `dist/` 파일을 새 소스로 빌드한 결과로 간주하지 않습니다.

1. [빌드 안내](BUILD.md)에 따라 현재 소스를 새 빌드 폴더에 컴파일합니다.
2. 실기에서 화면·터치·1D/2D 영점·CAL 유지·전원 동작을 확인합니다.
3. 신뢰하는 현재 빌드로 OS별 배포 파일을 생성합니다.

   ```sh
   python3 tools/package_native.py --build-dir build
   ```

   출력은 `dist/`에 생성됩니다. 같은 출력 이름이 있으면 중단하므로
   `--output dist/새로운배포폴더`로 분리할 수 있습니다.
   패키징은 버전 문자열과 보드·파티션을 검사하지만 소스와 바이너리의 동일성을 증명하지 않습니다.

4. 해당 커밋에 버전 태그를 만들고 GitHub Releases에 ZIP과 SHA-256을 첨부합니다.
   바이너리는 Git 소스 이력에 넣지 않습니다.
5. 배포에 포함되는 펌웨어·폰트·런타임의 고지와 라이선스 파일도 함께 포함합니다.

GitHub Actions는 호스트 회귀 테스트를 수행합니다. 현재 워크플로는 ESP32 펌웨어
컴파일·실기 업로드·릴리스 게시를 자동 실행하지 않습니다.
