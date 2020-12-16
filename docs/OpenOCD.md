# Prerequsites

Install dependencies and GDB:
```
sudo apt install autoconf automake libtool libusb-1.0-0-dev gdb-multiarch
```

## Building OpenOCD
Get the source:
```
git clone https://repo.or.cz/openocd.git
```

Save [this configuration](https://gist.githubusercontent.com/Daft-Freak/6662102321ee02260fc4634a63249c08/raw/956357984150c22720d484050f22c094a2744401/32blit.cfg) into `tcl/board/`

Build and install:
```
cd openocd

./bootstrap 
./configure --prefix=$HOME/install
make
make install
```

Install udev rules:
```
sudo cp contrib/60-openocd.rules /etc/udev/rules.d/
```

## Running OpenOCD

You should be able to run `~/install/bin/openocd -f board/32blit.cfg` and get output like:

```
Info : Listening on port 6666 for tcl connections
Info : Listening on port 4444 for telnet connections
Info : clock speed 1800 kHz
Info : STLINK V2J30M19 (API v2) VID:PID 0483:374B
Info : Target voltage: 0.010952
Error: target voltage may be too low for reliable debugging
Info : stm32h750vbz.cpu0: hardware has 8 breakpoints, 4 watchpoints
Info : starting gdb server for stm32h750vbz.cpu0 on 3333
Info : Listening on port 3333 for gdb connections
```

You can now connect GDB.

## VS Code

If you have followed the VS Code instructions, you can setup debugging in VS Code. Add a configuration like this to `launch.json`:
```json
{
    "version": "0.2.0",
    "configurations": [
        // other configurations...
        {
            "name": "(arm gdb) Launch",
            "type": "cppdbg",
            "request": "launch",
            "program": "${command:cmake.launchTargetPath}",
            "args": [],
            "stopAtEntry": false,
            "cwd": "${workspaceFolder}",
            "environment": [],
            "externalConsole": false,
            "MIMode": "gdb",
            "miDebuggerPath": "/usr/bin/gdb-multiarch",
            "debugServerPath": "/home/pi/install/bin/openocd",
            "debugServerArgs": "-f board/32blit.cfg -c init -c \"reset init\"",
            "filterStderr": true,
            "serverStarted": "target halted due to debug-request, current mode: Thread ",
            "serverLaunchTimeout": 60000,
            "setupCommands": [
                { "text": "-target-select remote localhost:3333", "description": "connect to target", "ignoreFailures": false },
                { "text": "-file-exec-and-symbols ${command:cmake.launchTargetPath}", "description": "load file", "ignoreFailures": false},
                { "text": "-interpreter-exec console \"monitor reset\"", "ignoreFailures": false },
                { "text": "-interpreter-exec console \"monitor halt\"", "ignoreFailures": false },
                { "text": "-target-download", "description": "flash target", "ignoreFailures": false }
            ]
        }
    ]
}
```

You will need to adjust `debugServerPath`. If you already have a configuration for a local GDB, add this one to the list.

Pressing `F5` will now flash and debug the current target.


### Debugging a Game

When you debug a game it will overwrite the start of the flash and be missing metadata, you can launch it by selecting "game @0" from the menu. You many need to manually press the reset button to start the game.

If you need to step into calls into the firmware, you will also need to flash a debug build of that and load the symbols.
```
-exec add-symbol-file /path/to/firmware/firmware.elf
```