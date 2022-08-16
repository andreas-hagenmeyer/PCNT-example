#include <Arduino.h>
#include "driver/pcnt.h"


// setting PWM properties
const int freq = 3000;
const int ledChannel = 1;
const int resolution = 8;

#define PinPwm 12
#define PinPcnt 35

uint32_t    overflow_cnt = 0;
uint16_t    pcntVal = 0;

pcnt_config_t pcnt_config = {
  .pulse_gpio_num = PinPcnt,
  .ctrl_gpio_num = -1,
  .lctrl_mode = PCNT_MODE_KEEP,
  .hctrl_mode = PCNT_MODE_KEEP,
  .pos_mode = PCNT_COUNT_INC,
  .neg_mode = PCNT_COUNT_DIS,
  .counter_h_lim = 3000,
  .counter_l_lim = 0,
  .unit = PCNT_UNIT_0,
  .channel = PCNT_CHANNEL_0
};



void IRAM_ATTR pcnt_event_handler(void *arg)
{
  overflow_cnt ++;
  PCNT.int_clr.val = BIT(PCNT_UNIT_0);
  Serial.println("* overflow ");
  
  
};

void pcnt_init();

void setup() {
  Serial.begin(115200);
  pcnt_init();
  

  ledcSetup(ledChannel, freq, resolution);
  ledcAttachPin(PinPwm, ledChannel);

  ledcWrite(ledChannel, 125);

  Serial.println("Setup finished");
}

void loop() {
  int i = 0;
  while(true)
  {
    i++;
  }
  
}

void pcnt_init()
{
  pinMode(PinPcnt,INPUT);
  pcnt_unit_config(&pcnt_config);

  pcnt_isr_register(pcnt_event_handler, NULL, 0, NULL);                   // Setup Register ISR handler
  pcnt_intr_enable(PCNT_UNIT_0);  

  pcnt_set_filter_value(PCNT_UNIT_0, 1023);                               // 1023 * 12,5 ns = 12,8 us
  pcnt_filter_enable(PCNT_UNIT_0); 

  pcnt_counter_pause(PCNT_UNIT_0);                                        // Pause PCNT unit
  pcnt_counter_clear(PCNT_UNIT_0);                                        // Clear PCNT unit
  
  pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_H_LIM);                         // Enable event to watch - max count
  pcnt_counter_resume(PCNT_UNIT_0);                                       // Resume PCNT unit - starts count


}