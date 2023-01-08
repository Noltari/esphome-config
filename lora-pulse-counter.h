#ifndef LORA_PULSE_COUNTER_H
#define LORA_PULSE_COUNTER_H

#include <SX127x.h>

#define LORA_BW 125000
#define LORA_CODE_RATE 5
#define LORA_CRC_ENABLE true
#define LORA_FREQ 869525000
#define LORA_HDR_TYPE SX127X_HEADER_IMPLICIT
#define LORA_PREAMBLE_LEN 8
#define LORA_SPREAD_FACTOR 7
#define LORA_SYNC_WORD 0x4C

#define HDR_SIZE 2
#define PULSE_SIZE 4
#define MISC_SIZE 3
#if defined(CRC16_ENABLE)
  #define CRC_SIZE 2
#else
  #define CRC_SIZE 0
#endif /* CRC16_ENABLE */

#define DATA_SIZE (HDR_SIZE + PULSE_SIZE + MISC_SIZE)
#define PKT_SIZE (DATA_SIZE + CRC_SIZE)

#define ADC_MASK 0x3FF
#define VCC_SHIFT 0
#define TEMP_SHIFT 10
#define PULSE_BIT (1UL << 20)

#define ADC_REF 3.36f
#define ADC_RES 1023
#define BAT_CNT 4.0f
#define GAS_CONV 100.0f

#define LORA_POLLING 100
#define LORA_START_INTERVAL 10000
#define LORA_WAIT 5

class LoRaSensors : public PollingComponent
{
  private:
    SX127x lora;
    unsigned long lora_millis;
    bool lora_on;

    int SS;
    int RST;

  public:
    Sensor *pulse_active_sensor = new Sensor();
    Sensor *pulse_sensor = new Sensor();
    Sensor *gas_sensor = new Sensor();
    Sensor *cell_volts_sensor = new Sensor();
    Sensor *bat_volts_sensor = new Sensor();
    Sensor *temp_sensor = new Sensor();
    Sensor *rssi_sensor = new Sensor();
    Sensor *snr_sensor = new Sensor();

    LoRaSensors(int SS, int RST) : PollingComponent(LORA_POLLING)
    {
      this->lora_millis = 0;
      this->lora_on = false;
      this->SS = SS;
      this->RST = RST;
    }

    float get_setup_priority(void) const override
    {
      return esphome::setup_priority::LATE;
    }

    void lora_start(void)
    {
      this->lora_millis = millis();

      if (this->lora.begin(this->SS, this->RST, -1, -1, -1))
      {
        this->lora_on = true;

        ESP_LOGI("LoRa", "LoRa started succesfully");

        this->lora.setFrequency(LORA_FREQ);

        this->lora.setRxGain(LORA_RX_GAIN_BOOSTED, SX127X_RX_GAIN_AUTO);

        this->lora.setSpreadingFactor(LORA_SPREAD_FACTOR);
        this->lora.setBandwidth(LORA_BW);
        this->lora.setCodeRate(LORA_CODE_RATE);

        this->lora.setHeaderType(LORA_HDR_TYPE);
        this->lora.setPreambleLength(LORA_PREAMBLE_LEN);
        this->lora.setPayloadLength(PKT_SIZE);
        this->lora.setCrcEnable(LORA_CRC_ENABLE);

        this->lora.setSyncWord(LORA_SYNC_WORD);

        this->lora.request(SX127X_RX_CONTINUOUS);
      }
      else
      {
        ESP_LOGE("LoRa", "LoRa start error!");
      }
    }

    void lora_rx(uint8_t *data, uint8_t len)
    {
      int off = 0;

      ESP_LOGD("LoRa", "LoRa: RX len=%d bytes=[%02X %02X %02X %02X %02X %02X %02X %02X %02X]",
        len, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8]);

      if (data[off++] == 'L' && data[off++] == 'A')
      {
        /* Pulse counter */
        uint32_t pulse_cnt = (data[off++] << 24);
        pulse_cnt |= (data[off++] << 16);
        pulse_cnt |= (data[off++] << 8);
        pulse_cnt |= (data[off++] << 0);

        /* Misc */
        uint32_t misc = (data[off++] << 16);
        misc |= (data[off++] << 8);
        misc |= (data[off++] << 0);

        float cell_volts = (((misc >> VCC_SHIFT) & ADC_MASK) * ADC_REF) / ADC_RES;
        bool pulse_active = !!(misc & PULSE_BIT);
        int temp = (misc >> TEMP_SHIFT) & ADC_MASK;

        /* LoRa signal */
        int16_t rssi = this->lora.packetRssi();
        float snr = this->lora.snr();

        /* CRC16 */
        #if defined(CRC16_ENABLE)
          uint16_t crc16_calc = esphome::crc16(data, DATA_SIZE);
          uint16_t crc16_lora = (data[DATA_SIZE] << 8) | (data[DATA_SIZE + 1] << 0);
          if (crc16_lora != crc16_calc)
          {
            ESP_LOGE("LoRa", "LoRa: CRC16 error RX=%04X vs Calc=%04X", crc16_lora, crc16_calc);
            return;
          }
        #endif /* CRC16_ENABLE */

        this->bat_volts_sensor->publish_state(cell_volts * BAT_CNT);
        this->cell_volts_sensor->publish_state(cell_volts);
        this->gas_sensor->publish_state(pulse_cnt / GAS_CONV);
        this->pulse_active_sensor->publish_state(pulse_active);
        this->pulse_sensor->publish_state(pulse_cnt);
        this->rssi_sensor->publish_state(rssi);
        this->snr_sensor->publish_state(snr);
        this->temp_sensor->publish_state(temp);
      }
    }

    void setup(void) override
    {
      this->lora_start();
    }

    void update(void) override
    {
      if (!this->lora_on)
      {
        if (millis() > this->lora_millis + LORA_START_INTERVAL)
        {
          this->lora_start();
        }
      }
      else
      {
        this->lora.wait(LORA_WAIT);

        uint8_t status = this->lora.status();
        if (status == SX127X_STATUS_RX_DONE)
        {
          int len = this->lora.available();
          if (len > 0)
          {
            uint8_t data[PKT_SIZE];

            ESP_LOGD("LoRa", "LoRa: RX len=%d", len);

            if (len > PKT_SIZE)
            {
              len = PKT_SIZE;
            }

            this->lora.read(data, len);
            this->lora_rx(data, len);
          }
        }

        this->lora.request(SX127X_RX_CONTINUOUS);
      }
    }
};

#endif /* LORA_PULSE_COUNTER_H */
