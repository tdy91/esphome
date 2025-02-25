#include "trv_pid.h"
#include "esphome/core/log.h"
#include <cmath>  // for std::isnan()

namespace esphome {
namespace trv_pid {

static const char *TAG = "trv_pid";

enum ErrorCodes {
  ERR_ROOM_TEMP_FROM_HA_UNAVAILABLE = 0,
  ERR_ROOM_TEMP_FROM_AM2302_UNAVAILABLE,
  ERR_ROOM_TEMP_UNAVAILABLE,
  ERR_TARGET_TEMP
};
#define SET_ERROR(e) (uError_ |= 1UL << e)
#define CLEAR_ERROR(e) (uError_ &= ~(1UL << e))
#define INVALID_TEMP -100

PIDCoefficients trv_pid::calculatePIDCoefficients_(float fKp, float fTi, float fTd, float fTe, float fN) {
  PIDCoefficients coeffs;
  coeffs.fCi = fKp * fTe / fTi;
  coeffs.fCd1 = 1 / (1 + fN * fTe / fTd);
  coeffs.fCd2 = fKp * fN / (1 + fN * fTe / fTd);
  return coeffs;
}

float calculatePIDCorrection_(float fError, float &fUiMem, float &fUdMem, float fCi, float fCd1, float fCd2) {
  fUiMem += fError;                        // Intégral
  fUdMem = fCd1 * fUdMem + fCd2 * fError;  // Dérivé
  return fCi * fError + fUiMem + fUdMem;   // Commande totale (Proportionnel + Intégral + Dérivé)
}

void TargetTemperatureNumber::setup() {
  float value;
  if (!this->restore_value_) {
    value = this->initial_value_;
  } else {
    this->pref_ = global_preferences->make_preference<float>(this->get_object_id_hash());
    if (!this->pref_.load(&value)) {
      if (!std::isnan(this->initial_value_)) {
        value = this->initial_value_;
      } else {
        value = this->traits.get_min_value();
      }
    }
  }
  this->publish_state(value);
}
void TargetTemperatureNumber::control(float value) {
  this->set_trigger_->trigger(value);

  this->publish_state(value);

  if (this->restore_value_)
    this->pref_.save(&value);
}

void TargetTemperatureNumber::dump_config() { LOG_NUMBER("", "Target Temperature Number", this); }

void trv_pid::set_target_temp_number(TargetTemperatureNumber *target_temp_number) {
  this->target_temp_number_ = target_temp_number;
}

void trv_pid::setup() {
  ESP_LOGD(TAG, "TRV PID Component Initialized");
  this->restore_state();
  pidCoefficients_int_ = calculatePIDCoefficients_(this->kp_, this->ti_, this->td_, this->sampleTime_s, this->n_);
  pidCoefficients_ext_ =
      calculatePIDCoefficients_(this->kp_ext_, this->ti_ext_, this->td_ext_, this->sampleTime_s, this->n_ext_);
}

void trv_pid::loop() {}

// update() will be called every "sample_time_s" seconds.
void trv_pid::update() {
  // Local variables declaration
  bool bSaturation1 = false, bSaturation1_ext = false, bSaturation2 = false;
  float fRoomTempMem, fCurrentExtTempMem;
  float fDegreCorrection = 0, fDegreCorrectionMax = 0, fDegreCorrection_ext = 0;
  float fError, fErrorMem, fError_ext, fErrorMem_ext;
  float fUi, fUiMem, fUi_ext, fUiMem_ext;
  float fUd, fUdMem, fUd_ext, fUdMem_ext;
  float fCi, fCi_ext;
  int iTrvCommand;
  update_room_temperature_();
  ESP_LOGD(TAG, "Update this->fTargetTemp_=%f this->fRoomTemp_=%f", this->fTargetTemp_, this->fRoomTemp_);
  ESP_LOGD(
      TAG,
      "Update this->thermal_load_correction_number_->state=%f this->forecast_wheather_correction_number_->state=%f",
      this->thermal_load_correction_number_->state, this->forecast_wheather_correction_number_->state);

  // ESP_LOGD(TAG, "Update kp=%f fCi=%f", this->kp_, this->pidCoefficients_int_.fCi);
  // ESP_LOGD(TAG, "Update kp_ext=%f fCi=%f", this->kp_ext_, this->pidCoefficients_ext_.fCi);
  // 1  this->fTargetTemp_ = this->target_temp_number_->state;
  this->fTargetTemp_ = this->target_temperature_number_->state;
  fDegreCorrection = 0;
  if ((this->fCurrentExtTemp_ != INVALID_TEMP) && (this->fCurrentExtTemp_ < this->heater_off_outdoor_temperature_)) {
    ESP_LOGD(TAG, "Current External Temperature is under %f °C", this->heater_off_outdoor_temperature_);
    state_.heater_on = true;

    // Compute error terms
    fError = this->fTargetTemp_ - fRoomTemp_;
    fError_ext = this->fTargetTemp_ - this->fCurrentExtTemp_;

    // Compute integral terms
    fUi = fUiMem + fCi * fErrorMem;
    fUi_ext = fUiMem_ext + fCi_ext * fErrorMem_ext;

    // NaN verifications
    if (isnan(fUi)) {
      fUi = 0.;
      fUiMem = 0.;
    }
    if (isnan(fUi_ext)) {
      fUi_ext = 0.;
      fUiMem_ext = 0.;
    }

    // Derivatives calculation des dérivées
    fUd = 0;
    fUd_ext = 0;
    if (isnan(fUd)) {
      fUd = 0.;
      fUdMem = 0.;
    }
    if (isnan(fUd_ext)) {
      fUd_ext = 0.;
      fUdMem_ext = 0.;
    }

    // Temperature corrections calculation
    fDegreCorrection = this->kp_ * fError + fUi + fUd;
    fDegreCorrection_ext = this->kp_ext_ * fError_ext;
    ESP_LOGD(TAG, "Update fDegreCorrection=%f fDegreCorrection_ext=%f", fDegreCorrection, fDegreCorrection_ext);

    // Updating memories
    fRoomTempMem = this->fRoomTemp_;
    fErrorMem = fError;
    fErrorMem_ext = fError_ext;
    fUdMem = fUd;
    fUdMem_ext = fUd_ext;

    // Saturation of corrections
    bSaturation1 = (fDegreCorrection <= -fDegreCorrectionMax) || (fDegreCorrection >= fDegreCorrectionMax);
    bSaturation1_ext = (fDegreCorrection_ext <= -fDegreCorrectionMax) || (fDegreCorrection_ext >= fDegreCorrectionMax);

    // TRV command calculation
    fTrvCommand = (26 - this->fTargetTemp_ + this->thermal_load_correction_number_->state +
                   this->forecast_wheather_correction_number_->state - fDegreCorrection - fDegreCorrection_ext) *
                  100 / 11;
    ESP_LOGD(TAG, "AAAA fTrvCommand=%f", fTrvCommand);

    // Anti-windup to avoid saturation
    bSaturation2 = (fTrvCommand <= 0.) || (fTrvCommand >= 100.);
    fTrvCommand = std::clamp(fTrvCommand, 0.0f, 100.0f);

    // Memories update if no saturation
    if (!bSaturation1 && !bSaturation2) {
      fUiMem = fUi;
    }
    if (!bSaturation1_ext && !bSaturation2) {
      fUiMem_ext = fUi_ext;
    }

  } else {
    ESP_LOGD(TAG, "Current External Temperature is above %f °C", this->heater_off_outdoor_temperature_);
    state_.heater_on = false;
    fTrvCommand = (this->fCurrentExtTemp_ >= this->heater_off_outdoor_temperature_ && this->fCurrentExtTemp_ <= 20.0)
                      ? 50.0f
                      : 0.0f;
  }

  // Updating outdor temperature memory
  if (this->fCurrentExtTemp_ != INVALID_TEMP) {
    fCurrentExtTempMem = this->fCurrentExtTemp_;
  }
  // Output conversion et update
  iTrvCommand = static_cast<int>(round(fTrvCommand));
  if (bTrv_command_off) {
    iTrvCommand = uiTrv_command_off_value;
  }
  ESP_LOGD(TAG, "iTrvCommand %i", iTrvCommand);
  //  id(trv_command).state = iTrvCommand;
  //  id(trv_output).turn_on();
  //  id(trv_output).set_level(iTrvCommand / 100.);
  if (this->trv_output_ != nullptr) {
    this->trv_output_->set_level(iTrvCommand / 100.);
  }
}

// void trv_pid::on_temperature_changed_(const std::string &state) {
void trv_pid::update_room_temperature_() {
  if (this->room_temperature_sensor_ != nullptr && !std::isnan(this->room_temperature_sensor_->state)) {
    this->fRoomTemp_ = this->room_temperature_sensor_->state;
    this->bRoomTemperatureFromHAavailable_ = true;

    CLEAR_ERROR(ERR_ROOM_TEMP_FROM_HA_UNAVAILABLE);
    CLEAR_ERROR(ERR_ROOM_TEMP_UNAVAILABLE);

    ESP_LOGD("TRV_PID", "Room temperature changed to %.2f, bRoomTemperatureFromHAavailable=%i", this->fRoomTemp_,
             this->bRoomTemperatureFromHAavailable_);
  } else {
    this->bRoomTemperatureFromHAavailable_ = false;

    SET_ERROR(ERR_ROOM_TEMP_FROM_HA_UNAVAILABLE);

    ESP_LOGD("TRV_PID", "Room temperature is NaN, keeping previous value %.2f, bRoomTemperatureFromHAavailable=%i",
             this->fRoomTemp_, this->bRoomTemperatureFromHAavailable_);
  }
}

void trv_pid::on_heater_outdoor_temperature_changed_(std::string &state) {
  if (this->outdoor_temperature_sensor_ == nullptr || std::isnan(this->outdoor_temperature_sensor_->state)) {
    this->fCurrentExtTemp_ = fPrevCurrentExtTemp_;
    ESP_LOGW(TAG, "Outdoor temperature is invalid or sensor is null, reverting to last known value: %.2f",
             this->fCurrentExtTemp_);
  } else {
    this->fCurrentExtTemp_ = this->outdoor_temperature_sensor_->state;
    ESP_LOGD(TAG, "Outdoor temperature has changed to %.2f", this->fCurrentExtTemp_);
  }
}

void trv_pid::save_state() {
  // Sauvegarder la commande actuelle
  if (!this->prefs_.save(&this->fTrvCommand)) {
    ESP_LOGW("trv_pid", "Failed to save state");
  } else {
    ESP_LOGD("trv_pid", "State saved: fTrvCommand = %f", this->fTrvCommand);
  }
}

void trv_pid::restore_state() {
  if (this->prefs_.load(&this->fTrvCommand)) {
    ESP_LOGD(TAG, "État restauré : fTrvCommand=%f", this->fTrvCommand);
  } else {
    ESP_LOGD(TAG, "Aucun état sauvegardé, utilisation de la valeur par défaut : 50.0");
    this->fTrvCommand = 50.0;
  }
}

}  // namespace trv_pid
}  // namespace esphome
