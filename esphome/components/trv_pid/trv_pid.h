#pragma once
/* Thermal Radiator Valve PID  (TRV PID)
This component will calculate TRV PID command
Parameters:
 * float fTe  : sample time, unit : seconds
 * float fKp  : proportional coefficient, unit : none
 * float fTi  : integral coefficient, unit : seconds
 * float fKaw : anti windup coefficient, unit : none
 * float fTd  : derivative coefficient, unit : seconds
 * float fN   : derivative filter coefficient, unit : none
Input :
 * float fTcurrent :
 * float fTtarget  :
Ourput:
 * float l : length, unit : mm

*/
#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/api/custom_api_device.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/slow_pwm/slow_pwm_output.h"

namespace esphome {
namespace trv_pid {

struct PIDConfig {
  float Kp, Ti, Td, N, Kaw;
};

struct PIDCoefficients {
  float fCi;
  float fCd1;
  float fCd2;
};

struct TRVState {
  bool command_off = false;
  bool saturation1 = false;
  bool saturation1_ext = false;
  bool saturation2 = false;
  bool heater_on = false;
};

class TargetTemperatureNumber : public number::Number, public Component {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::PROCESSOR; }

  Trigger<float> *get_set_trigger() const { return set_trigger_; }
  void set_initial_value(float initial_value) { initial_value_ = initial_value; }
  void set_restore_value(bool restore_value) { this->restore_value_ = restore_value; }

 protected:
  void control(float value) override;
  float initial_value_{NAN};
  bool restore_value_{true};
  Trigger<float> *set_trigger_ = new Trigger<float>();

  ESPPreferenceObject pref_;
};

class trv_pid : public PollingComponent, public esphome::api::CustomAPIDevice, public esphome::sensor::Sensor {
 public:
  uint32_t sampleTime_s;
  trv_pid(uint32_t sampleTime_s) : PollingComponent(sampleTime_s * 1000), sampleTime_s(sampleTime_s) {}
  void setup() override;
  void loop() override;
  void update() override;
  void set_target_temp_number(TargetTemperatureNumber *target_temp_number);
  void set_target_temperature_number(number::Number *target_temp) { this->target_temperature_number_ = target_temp; }
  void set_thermal_load_correction_number(number::Number *thermal_load_correction) {
    this->thermal_load_correction_number_ = thermal_load_correction;
  }
  void set_forecast_wheather_correction_number(number::Number *forecast_wheather_correction) {
    this->forecast_wheather_correction_number_ = forecast_wheather_correction;
  }
  void set_target_temp(float temp) { this->fTargetTemp_ = temp; }
  float get_target_temp() const { return this->fTargetTemp_; }
  float get_setup_priority() const override { return esphome::setup_priority::HARDWARE; }
  uint32_t getSampleTime() const { return sampleTime_s; }
  void set_room_temperature_sensor(sensor::Sensor *sensor) { this->room_temperature_sensor_ = sensor; }
  void set_outdoor_temperature_sensor(sensor::Sensor *sensor) { this->outdoor_temperature_sensor_ = sensor; }
  void set_heater_off_outdoor_temperature(float heater_off_outdoor_temperature) {
    this->heater_off_outdoor_temperature_ = heater_off_outdoor_temperature;
  }
  void set_trv_output(esphome::output::FloatOutput *output) { this->trv_output_ = output; }
  void set_kp(float kp) { this->kp_ = kp; }
  void set_ti(float ti) { this->ti_ = ti; }
  void set_td(float td) { this->td_ = td; }
  void set_n(float n) { this->n_ = n; }
  void set_kaw(float kaw) { this->kaw_ = kaw; }
  void set_kp_ext(float kp_ext) { this->kp_ext_ = kp_ext; }
  void set_ti_ext(float ti_ext) { this->ti_ext_ = ti_ext; }
  void set_td_ext(float td_ext) { this->td_ext_ = td_ext; }
  void set_n_ext(float n_ext) { this->n_ext_ = n_ext; }

  // Persitent and configurable variables
  float fTrvCommand = 50.0;  // Default initial command
  bool bTrv_command_off = false;
  float uiTrv_command_off_value = 0.0;

 private:
  sensor::Sensor *room_temperature_sensor_{nullptr};
  sensor::Sensor *outdoor_temperature_sensor_{nullptr};
  void save_state();
  void restore_state();
  float fTargetTemp_ = 19;
  float heater_off_outdoor_temperature_ = 11.0;
  float kp_ = 1.0;
  float ti_ = 1.0;
  float td_ = 0.0;
  float n_ = 10.0;
  float kaw_ = 0.1;
  float kp_ext_ = 1.0;
  float ti_ext_ = 1.0;
  float td_ext_ = 0.0;
  float n_ext_ = 10.0;
  float setpoint_{20.0};  // Default temperature target
  float integral_{0.0};
  float last_error_{0.0};
  PIDConfig pid_, pid_ext_;
  ESPPreferenceObject prefs_;
  PIDCoefficients calculatePIDCoefficients_(float fKp, float fTi, float fTd, float fTe, float fN);
  float calculatePIDCorrection_(float fError, float &fUiMem, float &fUdMem, float fCi, float fCd1, float fCd2);
  void on_heater_outdoor_temperature_changed_(std::string &state);
  float fCurrentExtTemp_ = 0.0;
  float fPrevCurrentExtTemp_ = 0.0;
  // void on_temperature_changed_(const std::string &state);
  void update_room_temperature_();
  float fRoomTemp_ = 0.0;
  bool bRoomTemperatureFromHAavailable_ = false;
  PIDCoefficients pidCoefficients_int_;
  PIDCoefficients pidCoefficients_ext_;
  uint uError_ = 0;
  TRVState state_;
  // Number components we'll present to the front end
  TargetTemperatureNumber *target_temp_number_{nullptr};
  number::Number *target_temperature_number_{nullptr};
  number::Number *thermal_load_correction_number_{nullptr};
  number::Number *forecast_wheather_correction_number_{nullptr};
  esphome::output::FloatOutput *trv_output_{nullptr};
  // float fTrvCommand{0.0};
};

}  // namespace trv_pid
}  // namespace esphome
