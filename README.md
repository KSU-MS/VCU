# VCU
The control logic for our electric car's vehicle control unit, the thing that commands the inverter to spin the motor

## Main Branch
This should be stable code that you can push to the car at any point, if it doesn't work go yell at @Chance


## Prerequisites

- [cmake](https://cmake.org/)
- [ninja](https://ninja-build.org/) (Optional, recommended)
- [Arm GNU Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)

The VCU uses a Teensy 4.1 microcontroller (IMXRT1062 MCU, a 32-bit ARM Cortex-M7). To build firmware, you need the Arm GNU Toolchain for the AArch32 bare-metal target (`arm-none-eabi`).

Below are OS-specific setup instructions:

---

### Windows

1. **Install CMake and Ninja**

   Download and install [CMake for Windows](https://cmake.org/download/)
   
   Optional: Download and install [Ninja](https://ninja-build.org/)

    - Add cmake and ninja to path
   To add CMake and Ninja to your PATH on Windows:

   - If you used the CMake installer, during installation you can select "Add CMake to the system PATH for all users" (recommended).
   - For Ninja, after extracting `ninja.exe`, place it in a folder (e.g., `C:\ninja`) and add that folder to your PATH.

   **To manually add to PATH:**

   1. Press `Win + S` and type "environment variables", then select "Edit the system environment variables".
   2. Click `Environment Variables...` in the System Properties window.
   3. Under `System variables`, find and select `Path`, then click `Edit`.
   4. Click `New` and add the folders containing `cmake.exe` (e.g., `C:\Program Files\CMake\bin`) and `ninja.exe` (e.g., `C:\ninja`).
   5. Click `OK` to close all dialog windows.

   Open a new Command Prompt and run:
   ```
   cmake --version
   ninja --version
   ```
   to verify they are installed and on your PATH.


2. **Download the Arm GNU Toolchain**

   [Windows 64 Bit Hosted Cross Toolchain (mingw-w64-x86_64)](https://developer.arm.com/-/media/Files/downloads/gnu/14.3.rel1/binrel/arm-gnu-toolchain-14.3.rel1-mingw-w64-x86_64-arm-none-eabi.zip)

3. **Extract the toolchain**  
   Unzip the toolchain to a location of your choice, e.g., `C:\arm-gnu-toolchain-14.3.rel1-mingw-w64-x86_64-arm-none-eabi`

4. **Update the Toolchain Path**  
   Edit `cmake/toolchains/teensy41.cmake` and set `COMPILERPATH` to the `bin` folder inside the extracted directory. For example:
   ```
   set(COMPILERPATH "C:/arm-gnu-toolchain-14.3.rel1-mingw-w64-x86_64-arm-none-eabi/bin/")
   ```

---

### Linux

1. **Install CMake and Ninja**

   Use your package manager or download from [CMake downloads](https://cmake.org/download/):

   __Ubuntu/Debian__
   ```
   sudo apt-get install cmake
   
   #optional
   sudo apt-get install ninja-build   # optional
   ```

   __Fedora__
   ```
   sudo dnf install cmake

   sudo dnf install ninja-build   # optional
   ```

2. **Download the Arm GNU Toolchain**

   [Linux 64 Bit Hosted Cross Toolchain (x86_64 Linux)](https://developer.arm.com/-/media/Files/downloads/gnu/14.3.rel1/binrel/arm-gnu-toolchain-14.3.rel1-x86_64-arm-none-eabi.tar.xz)

3. **Extract the toolchain**

   ```
   mkdir ~/tools
   tar -xf arm-gnu-toolchain-14.3.rel1-x86_64-arm-none-eabi.tar.xz -C ~/tools
   ```
   (or another directory of your choosing.)

4. **Update the Toolchain Path**

   Edit `cmake/toolchains/teensy41.cmake` and set `COMPILERPATH` to the toolchain's `bin` directory:
   ```
   set(COMPILERPATH "${HOME}/tools/arm-gnu-toolchain-14.3.rel1-x86_64-arm-none-eabi/bin/")
   ```

---

### macOS (Apple Silicon)

1. **Install CMake**

   Use [Homebrew](https://brew.sh/):
   ```
   brew install cmake
   ```
   Or download a binary from the [CMake downloads](https://cmake.org/download/)

2. **Download the Arm GNU Toolchain**

   [macOS Apple Silicon (arm64) Hosted Cross Toolchain](https://developer.arm.com/-/media/Files/downloads/gnu/14.3.rel1/binrel/arm-gnu-toolchain-14.3.rel1-darwin-arm64-arm-none-eabi.tar.xz)

3. **Extract the toolchain**

   ```
   tar -xf arm-gnu-toolchain-14.3.rel1-darwin-arm64-arm-none-eabi.tar.xz -C $HOME/tools
   ```
   (Or another directory of your choosing.)

4. **Update the Toolchain Path**

   Edit `cmake/toolchains/teensy41.cmake` and set `COMPILERPATH` to the `bin` folder:
   ```
   set(COMPILERPATH "${HOME}/tools/arm-gnu-toolchain-14.3.rel1-darwin-arm64-arm-none-eabi/bin/")
   ```



#### **For other host/target architectures, see [Arm GNU Toolchain Downloads](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)**

---

To add additional libraries or other arduino dependencies, add `import_arduino_library_git(NAME "LINK" BRANCH DIRECTORIES)` or `fetch_and_add_url_lib(NAME "LINK" DIRECTORIES)` to `CMakeLists.txt` at the project root

Then add the name of the library to `teensy_target_link_libraries(...)` in `src/CMakeLists.txt` __AFTER `firmware` BUT BEFORE ANY OTHER LIBRARY!!!__ The order is important for the compiler to properly link the libaries with the target.
`

To build the project, run `./scripts/build.sh` or `./scripts/build.bat`
To clean the build files, run `./scripts/clean.sh` or `./scripts/clean.bat`

---

## States
### ``STARTUP = 0`` <- Transient
  - This is when the VCU is trying to initialize everything, if something fails here its either the code on this guy or this specific board most likely

  `Next state -> TRACTIVE_SYSTEM_DISABLED`
  #### List of gates
  - The setup function

### ``TRACTIVE_SYSTEM_DISABLED = 1`` <- Gate
  - GLV is on, but not TSV, we are waiting for the reset button and the SDC to be closed, which we get over CAN from the TCU

  `Next state -> TRACTIVE_SYSTEM_PRECHARGING`
  #### List of gates
  - Reset button hit
  - CAN message from the TCU saying its good
  - CAN message from the 8CU saying its good
  - DC bus voltage > 360

### ``TRACTIVE_SYSTEM_ENERGIZED = 2`` <- Gate
  - TSV is up, but RTD button isn't pressed, waiting for the message from the dash to start sending torque requests

  Next state -> TRACTIVE_SYSTEM_ENABLED
  #### List of gates
  - CAN message from the TCU saying its good
  - CAN message from the 8CU saying its good
  - DC bus voltage > 360
  - Dash sends RTD button is high
  - Brake pedal is depressed more than 30%

### ``TRACTIVE_SYSTEM_ENABLED = 3`` <- Transient
  - We've enabled everything required to go fast, and got the green light from the driver, now we play the buzzer and setup anything else to go

  Next state -> READY_TO_DRIVE
  #### List of gates
  - CAN message from the TCU saying its good
  - CAN message from the 8CU saying its good
  - DC bus voltage > 360

### ``READY_TO_DRIVE = 4`` <- Gate
  - Hot 2 Go

  #### List of gates
  - CAN message from the TCU saying its bad
  - CAN message from the 8CU saying its bad 
  - DC bus voltage < 360


## Some structure things
### src
- ``main.cpp``
  - This guy is all of the top level logic, where all the overall systems are tying together to make the thing function.
    - The ADC stage updates all the readings the board needs
    - The CAN stage handles all the incoming messages, updating paramenters and forwarding etc.
    - The State machine stage is executing all the logic specific to our current state, and tries to change states

- ``vcu.cpp``
  - He is what actually checks to make sure you can go to the next state. If he says you can't then you won't. He also has the logic of what to do with messages and sending all of the board specific messages.

- ``inverter.cpp``
  - Has the logic to make the motor spin, including sending messages and ensuring the power limit.

- ``accumulator.cpp``
  - The more or less just packing/unpacking messages for the acc.

### include
- ``main.hpp``
  - This has all of the class inits and imports needed for everything to function.

- ``vcu.hpp``
  - He defines what all the states are, and functions that let you change around states.

- ``inverter.hpp``
  - Defines the class that manages all the things to make the inverter spin.

- ``accumulator.hpp``
  - A class to hold all the important info from the accumulator that gets used.

- ``pedal_handeler.hpp``
  - Defines and implements all the logic needed for the pedals as required by FSAE rules.

- ``parameters.hpp``
  - Contains all of the defines for what pins are connected to what for all the micro/pcb specific stuff.


## Things to do
- [ ] Overhall the dash->vcu message system to seperate the two more
- [ ] Get distance tracking back with EEPROM memory (maybe add to the logger implementation)
- [ ] Make the BSPD fella more real
- [ ] Finish wheel speed implementation
- [ ] Drop in the launch control algo
- [ ] Get power effeciency math back in there
- [ ] Add support for digitalWrite to ADC.hpp or find some way to wrap it that isn't aids, same thing with the one delay
- [ ] Make pretty flow charts for everything
- [ ] Finish settings lib

## CAN DOC
We currently handel these messages
##### Unpack
  - ACU_SHUTDOWN_STATUS
  - PRECHARGE_STATUS
  - DASH_BUTTONS
##### Pack
  - VCU_PEDALS_TRAVEL
  - VCU_PEDAL_READINGS
  - VCU_STATUS
  - VCU_FIRMWARE_VERSION
  - BMS_CURRENT_LIMIT
  - M192_COMMAND_MESSAGE

I need to add
##### Unpack
  - VectorNav things
  - Torque mode setting stuff
##### Pack
  - VCU_BOARD_READINGS_ONE
  - VCU_BOARD_READINGS_TWO
  - Launch control stuff
