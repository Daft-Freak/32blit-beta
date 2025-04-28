VS Code cmake-tools-kits.json
```json
[
  {
    "name": "ESP32-C3",
    "environmentSetupScript": "${env:HOME}/repos/esp/v5.4.1/esp-idf/export.sh",
    "toolchainFile": "${env:HOME}/repos/esp/v5.4.1/esp-idf/tools/cmake/toolchain-esp32c3.cmake",
    "cmakeSettings": {
      "ESP_TARGET": "esp32c3"
    },
    "isTrusted": true
  }
]
```


plain cmd line
```
source $HOME/repos/esp/v5.4.1/esp-idf/export.sh
cmake -DCMAKE_TOOLCHAIN_FILE=$IDF_PATH/tools/cmake/toolchain-esp32c3.cmake -DESP_TARGET:STRING=esp32c3
```

Add -DFLASH_PORT=/dev/ttyWhatever to use [name].flash targets


monitor
```
python -m esp_idf_monitor -p /dev/ttyUSB0 examples/serial-debug/serial-debug
```
