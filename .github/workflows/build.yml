name: build

on:
  push:
    paths: ["**/*.ino", ".github/workflows/build.yml"]
  workflow_dispatch:

jobs:
  build:
    runs-on: ubuntu-latest
    strategy:
      fail-fast: false
      matrix:
        t:
          - { name: ice, sketch: ice_ble_uart/ice_ble_uart.ino, fqbn: "esp32:esp32:esp32s3" }
          # add your other sketches later if you like:
          # - { name: smoke, sketch: smoke_chain_s3/smoke_chain_s3.ino, fqbn: "esp32:esp32:esp32s3" }
          # - { name: ui,    sketch: ui_web_s3_ap/ui_web_s3_ap.ino,     fqbn: "esp32:esp32:esp32s3" }

    steps:
      - uses: actions/checkout@v4

      - name: Set up Arduino CLI
        uses: arduino/setup-arduino-cli@v1

      - name: Install ESP32 core
        run: |
          arduino-cli core update-index
          # You can pin a version (e.g., esp32:esp32@3.0.7) if you want reproducible builds:
          arduino-cli core install esp32:esp32
          sudo apt-get update
          sudo apt-get install -y jq zip

      - name: Compile & bundle ${{ matrix.t.name }}
        env:
          FQBN:   ${{ matrix.t.fqbn }}
          SKETCH: ${{ matrix.t.sketch }}
          NAME:   ${{ matrix.t.name }}
        run: |
          set -e

          # Compile and export binaries
          arduino-cli compile --fqbn "$FQBN" --export-binaries "$SKETCH"

          # Find Arduino's build output path
          BUILD="$(arduino-cli compile --fqbn "$FQBN" --show-properties "$SKETCH" | sed -n 's/^build.path=//p' | tail -n1)"
          echo "BUILD=$BUILD"
          ls -la "$BUILD" || true

          mkdir -p out/$NAME

          # Collect expected artifacts from build path
          APP="$(ls "$BUILD"/*.ino.bin 2>/dev/null | head -n1)"
          PART="$(ls "$BUILD"/*partitions.bin 2>/dev/null | head -n1)"
          BOOT="$(ls "$BUILD"/*.bootloader.bin 2>/dev/null | head -n1 || true)"
          if [ -z "$BOOT" ]; then BOOT="$(find "$BUILD" -name 'bootloader*.bin' -print -quit)"; fi

          # boot_app0 comes from the installed ESP32 core
          DATA_DIR="$(arduino-cli config dump --format json | jq -r '.directories.data')"
          APP0="$(find "$DATA_DIR/packages/esp32" -type f -name 'boot_app0.bin' -print -quit)"

          : "${APP:?app .ino.bin not found}"
          : "${PART:?partitions.bin not found}"
          : "${BOOT:?bootloader.bin not found}"
          : "${APP0:?boot_app0.bin not found}"

          cp "$APP"  out/$NAME/firmware.bin
          cp "$PART" out/$NAME/partitions.bin
          cp "$BOOT" out/$NAME/bootloader.bin
          cp "$APP0" out/$NAME/boot_app0.bin

          # Offsets for typical ESP32-S3 layout (works with Arduino core defaults)
          cat > out/$NAME/flash_offsets.txt <<'EOF'
          bootloader.bin  0x0000
          partitions.bin  0x8000
          boot_app0.bin   0xE000
          firmware.bin    0x10000
          EOF

          (cd out/$NAME && zip -9 ../"${NAME}_bins".zip bootloader.bin partitions.bin boot_app0.bin firmware.bin flash_offsets.txt)

      - name: Upload artifacts
        uses: actions/upload-artifact@v4
        with:
          name: ${{ matrix.t.name }}_bins
          path: out/${{ matrix.t.name }}/*
