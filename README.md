# VCU
The control logic for our electric car's vehicle control unit
## WIP
Not making a RTOS based system yet, I don't think I am going to be doing too much zesty shit to *need* it

## States
- ``STARTUP = 0`` <- Transient
  - This is when the VCU is trying to initialize everything, if something fails here its either the code on this guy or this specific board most likely
  List of gates
  - The setup function

  Next state -> TRACTIVE_SYSTEM_DISABLED

- ``TRACTIVE_SYSTEM_DISABLED = 1`` <- Gate
  - GLV is on, but not TSV, we are waiting for the reset button and the SDC to be closed, which we get over CAN from the TCU
  List of gates
  - Reset button hit
  - CAN message from the TCU

  Next state -> TRACTIVE_SYSTEM_PRECHARGING

- ``TRACTIVE_SYSTEM_PRECHARGING = 2`` <- Transient
  - TSV is comin', and assuming the TCU doesn't explode we should be good
  List of gates
  - CAN message from the TCU saying its good
  - CAN message from the 8CU saying its good

  Next state -> TRACTIVE_SYSTEM_ENERGIZED

- ``TRACTIVE_SYSTEM_ENERGIZED = 3`` <- Gate
  - TSV is up, but RTD button isn't pressed, waiting for the message from the dash to start sending torque requests
  List of gates
  - CAN message from the TCU saying its good
  - CAN message from the 8CU saying its good
  - Dash sends RTD button is high
  - Brake pedal is depressed

  Next state -> TRACTIVE_SYSTEM_ENABLED

- ``TRACTIVE_SYSTEM_ENABLED = 4`` <- Transient
  - We've enabled everything required to go fast, and got the green light from the driver, now we play the buzzer and setup anything else to go
  List of gates
  - CAN message from the TCU saying its good
  - CAN message from the 8CU saying its good
  - Buzzer goes high

- ``READY_TO_DRIVE = 5`` <- Gate
  - Hot 2 Go
  List of gates
  - CAN message from the TCU saying its good
  - CAN message from the 8CU saying its good
  - Buzzer goes low


## Some structure things
### src
- ``main.cpp``
  - This guy is all of the top level logic, where all the overall systems are tying together to make the thing function

- ``vcu.cpp``
  - He is what actually checks to make sure you can go to the next state, and if you cant he gives you the error.

- ``logger.cpp``
  - A platform agnostic way of doing debug info, tho it currently really only supports AVR things

- ``settings.cpp``
  - Provides logic to configure diffrent parameters of the VCU (major WIP)

### include
- ``main.hpp``
  - This has all of the class inits and imports needed for everything to function

- ``vcu.hpp``
  - He defines what all the states are, and functions that let you change around

- ``logger.cpp``
  - The class defs and inits for platform agnostic debug prints

- ``settings.hpp``
  - All the defs needed for parameter configuration of the VCU (major WIP)

- ``Pin_Defs.hpp``
  - Contains all of the defines for what pins are connected to what for all the micro/pcb specific stuff.


## Things to do
- [x] Make the pedal handeler
- [x] Get the inverter class goin
- [x] Pack things into CAN messages, ideally use the auto generated lib
- [ ] Test can_tools.hpp and logger.hpp
- [ ] Add support for digitalWrite to ADC or find some way to wrap it that isn't aids
- [ ] Make pretty flow charts for everything
- [ ] Finish wheel speed implementation
- [ ] Get distance tracking back with EEPROM memory (maybe add to the logger implementation)
- [ ] Finish settings lib
- [ ] Get can_tools.hpp, logger.hpp, and maybe settings.hpp all pushed to ksu-fw-common
