#pragma once

#include "esphome/core/component.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/uart/uart.h"

namespace esphome {
namespace uart_mailbox_notifier {

class Uart_Mailbox_Notifier : public Component,public text_sensor::TextSensor,public uart::UARTDevice {
public:
  void loop() override {
    while (available()) {
      uint8_t c;
      read_byte(&c);

      // Beispiel: 0x55 = true, 0xAA = false
      if (c == 0x55) {
        publish_state("true");
      } else if (c == 0xAA) {
        publish_state("false");
      }
    }
  }
};

}  // namespace uart_mailbox_notifier
}  // namespace esphome
