#include "main.hpp"

void setup() {
  consol.logln("Booting...");

  vcu.begin();

  consol.logln("Booted");
}

void loop() {
  //
  //// ADC Stage
  // This is the only part of the loop that touches real hardware, everything
  // after it lives in VCU::tick so it can be run in the simulator
  apps1.update();
  apps2.update();
  bse.update();
  steering_angle.update();
  vcu.pedals->update_travel(apps1.value.in, apps2.value.in, bse.value.in);
  vcu.pedals->update_steering(steering_angle.value.in);

  vsense_bspd.update();
  vsense_bspd_bse.update();
  vsense_bspd_hall.update();
  vcu.update_bspd(vsense_bspd.value.in, vsense_bspd_bse.value.in,
                  vsense_bspd_hall.value.in);

  vcu.tick(millis());
}
