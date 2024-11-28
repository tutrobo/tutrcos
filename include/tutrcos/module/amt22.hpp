#pragma once

#include "main.h"

#include <cassert>
#include <cmath>
#include <vector>

extern SPI_HandleTypeDef hspi2;

/* sample (AIO_Board)
#include "amt22.hpp"
#include "mbed.h"
#include "uart485.hpp"

#define SAM_TIME 0.01

UnlockedSerial pc(USBTX, USBRX, 921600);
UnlockedSPI spi(SPI1MOSI, SPI1MISO, SPI1SCK);
Encoder *enc = new Amt22(&spi, SPI1CS1, Encoder::BitSize::_14),

int main(){
    while(true){
        enc->update(SAM_TIME);
        pc.printf("%f\n", enc.getDegree());
        wait(SAM_TIME);
    }
}
*/

class AMT22 {
public:
  AMT22(uint8_t bitsize, bool is_reverse = false, float reduction_ratio = 1,
        bool singleturn = false, float cut_point = 0.5)
      : bits_(bitsize), singleturn_(singleturn),
        cut_point_((1 << bitsize) * cut_point), ppr_(1 << bitsize),
        dir_((is_reverse) ? -1 : 1), reduction_ratio_(reduction_ratio),
        offset_count_(0 / 360. * ppr_ / dir_ / reduction_ratio) {
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
    // spi_->format(8);
    // dout_ = 1;
    // tim_update_.start();
  }

  bool update(float dt = -1) {
    if (dt < 0) {
      // dt = tim_update_.elapsed_time().count() / 1000000.;
      // tim_update_.reset();
    }
    int32_t value;
    uint8_t tmp_bits = 14 - bits_;
    std::vector<char> rxdata(2);

    rxdata = send({0x00, 0x00});

    if (checksum(rxdata[0], rxdata[1]))
      return 1;

    raw_count_ = value =
        ((rxdata[1] & 0x3F) << (8 - tmp_bits)) | (rxdata[0] >> tmp_bits);

    if (singleturn_) {
      if (value > cut_point_)
        count_ = value - ppr_;
      else
        count_ = value;
      // count_ = value;
    } else {
      if ((value - pre_value_) < -(ppr_ / 2))
        revolution_++;
      if ((value - pre_value_) > (ppr_ / 2))
        revolution_--;
      count_ = revolution_ * ppr_ + value;
    }

    if (dt > 0) {
      cps_ = (count_ - pre_count_) / dt;
    }

    pre_value_ = value;
    pre_count_ = count_;
    return 0;
  }

  void setZeroPoint() {
    send({0x00, 0x70});
    count_ = pre_count_ = revolution_ = pre_value_ = 0;
  }

  int16_t getRawCount() { return raw_count_; }
  int32_t getCount() { return (count_ + offset_count_) * dir_; }
  float getRotation() {
    return static_cast<float>(getCount()) / ppr_ * reduction_ratio_;
  }
  float getDegree() { return getRotation() * 360.; }
  float getRad() { return getRotation() * 2 * M_PI; }

  float getCps() { return cps_ * dir_ * reduction_ratio_; }
  float getRps() { return getCps() / ppr_; }
  float getRpm() { return getRps() * 60; }
  float getDegreeps() { return getRps() * 360.; }
  float getRadps() { return getRps() * 2. * M_PI; }

  int8_t getDir() { return (getCps() < 0) ? -1 : (getCps() > 0) ? 1 : 0; }
  float getRedu() { return reduction_ratio_; }

private:
  // UnlockedSPI *spi_;
  // DigitalOut dout_;
  const uint16_t bits_;
  const bool singleturn_;
  const uint16_t cut_point_;

  // Timer tim_update_;
  bool once_flag_ = true;
  float cps_;
  int32_t pre_value_ = 0, raw_count_ = 0, revolution_ = 0;
  int32_t count_ = 0, pre_count_ = 0;
  const uint16_t ppr_;
  const int8_t dir_;
  const float reduction_ratio_;
  const int32_t offset_count_;

  std::vector<char> send(const std::vector<char> &data) {
    std::vector<char> rxdata(2);
    // __disable_irq();
    // dout_ = 0;
    // wait_us(3);
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)data.data(),
                            (uint8_t *)rxdata.data(), 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
    // for (const char c : data) {
    //  rxdata.emplace_back(spi_->write(c));
    //   wait_us(3);
    //}
    // dout_ = 1;
    //  __enable_irq();
    std::reverse(rxdata.begin(), rxdata.end());
    return rxdata;
  }

  bool checksum(uint8_t low_byte, uint8_t high_byte) {
    union checkbit {
      bool b[16];
      uint16_t B;
    };
    checkbit bit_;
    bool k0 = 0, k1 = 0;
    for (int i = 2; i < 16; i++) {
      if (i % 2 == 0) {
        k1 ^= bit_.b[i];
      } else {
        k0 ^= bit_.b[i];
      }
    }
    k0 = !k0;
    k1 = !k1;

    return (k1 == bit_.b[0]) & (k0 == bit_.b[1]);
  }
};
