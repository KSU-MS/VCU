# VCU
The control logic for our electric car's vehicle control unit, the thing that commands the inverter to spin the motor

## Main Branch
This should be stable code that you can push to the car at any point, if it doesn't work go yell at @Chance

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
