#pragma once

#include "loraDef.h"
#include <string>
#include <memory>
#include <functional>
#include "driver/spi_master.h"
#include "driver/gpio.h"

static constexpr const char *TAGLORA = "LORA";

typedef struct
{
  spi_host_device_t spiHost;
  gpio_num_t gpioMISO;
  gpio_num_t gpioMOSI;
  gpio_num_t gpioSCL;
  gpio_num_t gpioCS;
  gpio_num_t gpioRST;
  long loraFrequency;
  bool crc;
  int codingRate;
  int bandwith;
  int spreadingFactor;
} LoraServiceConfig;

class LoraService
{
private:
  spi_device_handle_t spi;
  LoraServiceConfig config;

  int implicit;
  long frequency;
  int send_packet_lost;

  void writeReg(uint8_t reg, uint8_t val);
  void writeRegBuffer(uint8_t reg, uint8_t *val, size_t len);
  int readReg(uint8_t reg);
  void readRegBuffer(uint8_t reg, uint8_t *val, size_t len);

  void reset(void);
  void explicitHeaderMode(void);
  void implicitHeaderMode(int size);

  void receive(void);
  int getIrq(void);
  void setDioMapping(int dio, int mode);
  int getDioMapping(int dio);
  void setPreambleLength(long length);
  long getPreambleLength(void);
  void setSyncWord(int sw);

  void dumpRegisters(void);

  // static void onEvent(void *pvParameters);

  // using OnReceiveHandler = std::function<void(std::string data)>;
  // OnReceiveHandler onReceive;

public:
  LoraService(LoraServiceConfig config);
  ~LoraService();

  void sleep(void);
  void idle(void);
  void close(void);
  int init(void);
  int isInitialized(void);
  void sendPacket(std::vector<uint8_t> buffer);
  int receivePacket(uint8_t *buf, int size);
  bool hasPacket(void);

  void setBandwidth(int sbw);
  void setCodingRate(int denominator);
  void setCrc(bool);
  void setFrequency(long frequency);
  void setSpreadingFactor(int sf);
  void setTxPower(int level);
  int getBandwidth(void);
  int getCodingRate(void);
  int getSpreadingFactor(void);
  int getPacketLost(void);
  int getPacketRSSI(void);
  float getPacketSNR(void);
};
