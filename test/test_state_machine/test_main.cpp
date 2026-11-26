#include <sim_test_main.hpp>
#include <sim_car.hpp>

TEST_CASE("boots into TRACTIVE_SYSTEM_DISABLED") {
  sim::Car car;
  car.run_for(10);
  CHECK(car.vcu.get_current_state() == TRACTIVE_SYSTEM_DISABLED);
}
