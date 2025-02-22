# VCU
The control logic for our electric car's vehicle control unit

## WIP
Not making a RTOS based system yet, I don't think I am going to be doing too much zesty shit to *need* it

## Things to do
- [ ] Make the pedal handeler
- [ ] Get the inverter class goin
- [ ] Pack things into CAN messages, ideally use the auto generated lib
- [ ] Test can_tools.hpp and logger.hpp
- [ ] Add support for digitalWrite to ADC or find some way to wrap it that isn't aids
- [ ] Make pretty flow charts for everything
- [ ] Finish wheel speed implementation
- [ ] Get distance tracking back with EEPROM memory (maybe add to the logger implementation)
- [ ] Finish settings lib
- [ ] Get can_tools.hpp, logger.hpp, and maybe settings.hpp all pushed to ksu-fw-common
