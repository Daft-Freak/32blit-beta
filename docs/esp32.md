## VS Code cmake-tools-kits.json
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
  },
  {
    "name": "ESP32-P4",
    "environmentSetupScript": "${env:HOME}/repos/esp/v5.4.1/esp-idf/export.sh",
    "toolchainFile": "${env:HOME}/repos/esp/v5.4.1/esp-idf/tools/cmake/toolchain-esp32p4.cmake",
    "cmakeSettings": {
      "ESP_TARGET": "esp32p4"
    },
    "isTrusted": true
  }
]
```

## Debugging (kinda)

launch.json (`configurations`)
```json
{
    "name": "ESP32-P4 Debug",
    "type": "cppdbg",
    "request": "launch",
    "program": "${command:cmake.launchTargetPath}",
    "args": [],
    "stopAtEntry": false,
    "cwd": "${workspaceFolder}",
    "environment": [],
    "MIMode": "gdb",
    "miDebuggerPath": "/home/daftfreak/.espressif/tools/riscv32-esp-elf-gdb/14.2_20240403/riscv32-esp-elf-gdb/bin/riscv32-esp-elf-gdb",
    "debugServerPath": "/home/daftfreak/.espressif/tools/openocd-esp32/v0.12.0-esp32-20241016/openocd-esp32/bin/openocd",
    // disable multicore debug
    "debugServerArgs": "-c \"set ESP_ONLYCPU 1\" -f board/esp32p4-builtin.cfg",
    "filterStderr": true,
    "serverStarted": "Listening on port 3333 for gdb connections",
    "serverLaunchTimeout": 10000,
    "setupCommands": [
        { "text": "-target-select remote localhost:3333", "description": "connect to target", "ignoreFailures": false },
        { "text": "-file-exec-and-symbols ${command:cmake.launchTargetPath}", "description": "load file", "ignoreFailures": false},
        { "text": "-interpreter-exec console \"monitor reset\"", "ignoreFailures": false },
        { "text": "-interpreter-exec console \"monitor halt\"", "ignoreFailures": false },
    ],
    "preLaunchTask": "esp32-flash"
},
```

tasks.json

```json
{
    "version": "2.0.0",
    "tasks": [
        {
            "label": "esp32-flash",
            "type": "shell",
            "command": "ninja ${command:cmake.launchTargetName}.flash",
            "options": {
                "cwd": "${command:cmake.buildDirectory}",
            },
        }
    ]
}
```

## plain cmd line
```
source $HOME/repos/esp/v5.4.1/esp-idf/export.sh
cmake -DCMAKE_TOOLCHAIN_FILE=$IDF_PATH/tools/cmake/toolchain-esp32c3.cmake -DESP_TARGET:STRING=esp32c3
```
(or esp32p4, ...)

Add -DFLASH_PORT=/dev/ttyWhatever to use [name].flash targets


monitor
```
python -m esp_idf_monitor -p /dev/ttyUSB0 examples/serial-debug/serial-debug
```
