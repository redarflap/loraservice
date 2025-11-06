
#include <string.h>
#include "esp_log.h"
#include <esp_err.h>
#include "loraService.h"
#include "loraDef.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

/**
 * Write a value to a register.  * @param reg Register index.
 * @param val Value to write.
 */
void LoraService::writeReg(uint8_t reg, uint8_t val)
{
  uint8_t out[2] = {static_cast<uint8_t>(0x80 | reg), val};
  uint8_t in[2];

  spi_transaction_t t = {
      .flags = 0,
      .length = 8 * sizeof(out),
      .tx_buffer = out,
      .rx_buffer = in};

  spi_device_transmit(spi, &t);
}

/**
 * Write a buffer to a register.
 * @param reg Register index.
 * @param val Value to write.
 * @param len Byte length to write.
 */
void LoraService::writeRegBuffer(uint8_t reg, uint8_t *val, size_t len)
{
  uint8_t *out;
  out = (uint8_t *)malloc(len + 1);
  out[0] = 0x80 | reg;
  for (int i = 0; i < len; i++)
  {
    out[i + 1] = val[i];
  }

  spi_transaction_t t = {
      .flags = 0,
      .length = 8 * (len + 1),
      .tx_buffer = out,
      .rx_buffer = NULL};

  spi_device_transmit(spi, &t);
  free(out);
}

/**
 * Read the current value of a register.
 * @param reg Register index.
 * @return Value of the register.
 */
int LoraService::readReg(uint8_t reg)
{
  uint8_t out[2] = {reg, 0xff};
  uint8_t in[2];

  spi_transaction_t t = {
      .flags = 0,
      .length = 8 * sizeof(out),
      .tx_buffer = out,
      .rx_buffer = in};

  spi_device_transmit(spi, &t);
  return in[1];
}

/**
 * Read the current value of a register.
 * @param reg Register index.
 * @return Value of the register.
 * @param len Byte length to read.
 */
void LoraService::readRegBuffer(uint8_t reg, uint8_t *val, size_t len)
{
  uint8_t *out;
  uint8_t *in;
  out = (uint8_t *)malloc(len + 1);
  in = (uint8_t *)malloc(len + 1);
  out[0] = reg;
  for (int i = 0; i < len; i++)
  {
    out[i + 1] = 0xff;
  }

  spi_transaction_t t = {
      .flags = 0,
      .length = 8 * (len + 1),
      .tx_buffer = out,
      .rx_buffer = in};

  spi_device_transmit(spi, &t);

  for (int i = 0; i < len; i++)
  {
    val[i] = in[i + 1];
  }
  free(out);
  free(in);
}

/**
 * Perform physical reset on the Lora chip
 */
void LoraService::reset(void)
{
  gpio_set_level(config.gpioRST, 0);
  vTaskDelay(pdMS_TO_TICKS(1));
  gpio_set_level(config.gpioRST, 1);
  vTaskDelay(pdMS_TO_TICKS(10));
}

/**
 * Configure explicit header mode.
 * Packet size will be included in the frame.
 */
void LoraService::explicitHeaderMode(void)
{
  implicit = 0;
  writeReg(REG_MODEM_CONFIG_1, readReg(REG_MODEM_CONFIG_1) & 0xfe);
}

/**
 * Configure implicit header mode.
 * All packets will have a predefined size.
 * @param size Size of the packets.
 */
void LoraService::implicitHeaderMode(int size)
{
  implicit = 1;
  writeReg(REG_MODEM_CONFIG_1, readReg(REG_MODEM_CONFIG_1) | 0x01);
  writeReg(REG_PAYLOAD_LENGTH, size);
}

/**
 * Sets the radio transceiver in idle mode.
 * Must be used to change registers and access the FIFO.
 */
void LoraService::idle(void)
{
  writeReg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

/**
 * Sets the radio transceiver in sleep mode.
 * Low power consumption and FIFO is lost.
 */
void LoraService::sleep(void)
{
  writeReg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

/**
 * Sets the radio transceiver in receive mode.
 * Incoming packets will be received.
 */
void LoraService::receive(void)
{
  writeReg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

/**
 * Configure power level for transmission
 * @param level 2-17, from least to most power
 */
void LoraService::setTxPower(int level)
{
  // RF9x module uses PA_BOOST pin
  if (level < 2)
    level = 2;
  else if (level > 17)
    level = 17;
  writeReg(REG_PA_CONFIG, PA_BOOST | (level - 2));
}

/**
 * Set carrier frequency.
 * @param frequency Frequency in Hz
 */
void LoraService::setFrequency(long _frequency)
{
  frequency = _frequency;

  uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

  writeReg(REG_FRF_MSB, (uint8_t)(frf >> 16));
  writeReg(REG_FRF_MID, (uint8_t)(frf >> 8));
  writeReg(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

/**
 * Set spreading factor.
 * @param sf 6-12, Spreading factor to use.
 */
void LoraService::setSpreadingFactor(int sf)
{
  if (sf < 6)
    sf = 6;
  else if (sf > 12)
    sf = 12;

  if (sf == 6)
  {
    writeReg(REG_DETECTION_OPTIMIZE, 0xc5);
    writeReg(REG_DETECTION_THRESHOLD, 0x0c);
  }
  else
  {
    writeReg(REG_DETECTION_OPTIMIZE, 0xc3);
    writeReg(REG_DETECTION_THRESHOLD, 0x0a);
  }

  writeReg(REG_MODEM_CONFIG_2, (readReg(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
}

/**
 * Get spreading factor.
 */
int LoraService::getSpreadingFactor(void)
{
  return (readReg(REG_MODEM_CONFIG_2) >> 4);
}

/**
 * Set Mapping of pins DIO0 to DIO5
 * @param dio Number of DIO(0 to 5)
 * @param mode mode of DIO(0 to 3)
 */
void LoraService::setDioMapping(int dio, int mode)
{
  if (dio < 4)
  {
    int _mode = readReg(REG_DIO_MAPPING_1);
    if (dio == 0)
    {
      _mode = _mode & 0x3F;
      _mode = _mode | (mode << 6);
    }
    else if (dio == 1)
    {
      _mode = _mode & 0xCF;
      _mode = _mode | (mode << 4);
    }
    else if (dio == 2)
    {
      _mode = _mode & 0xF3;
      _mode = _mode | (mode << 2);
    }
    else if (dio == 3)
    {
      _mode = _mode & 0xFC;
      _mode = _mode | mode;
    }
    writeReg(REG_DIO_MAPPING_1, _mode);
    ESP_LOGD(TAGLORA, "REG_DIO_MAPPING_1=0x%02x", _mode);
  }
  else if (dio < 6)
  {
    int _mode = readReg(REG_DIO_MAPPING_2);
    if (dio == 4)
    {
      _mode = _mode & 0x3F;
      _mode = _mode | (mode << 6);
    }
    else if (dio == 5)
    {
      _mode = _mode & 0xCF;
      _mode = _mode | (mode << 4);
    }
    ESP_LOGD(TAGLORA, "REG_DIO_MAPPING_2=0x%02x", _mode);
    writeReg(REG_DIO_MAPPING_2, _mode);
  }
}

/**
 * Get Mapping of pins DIO0 to DIO5
 * @param dio Number of DIO(0 to 5)
 */
int LoraService::getDioMapping(int dio)
{
  if (dio < 4)
  {
    int _mode = readReg(REG_DIO_MAPPING_1);
    ESP_LOGD(TAGLORA, "REG_DIO_MAPPING_1=0x%02x", _mode);
    if (dio == 0)
    {
      return ((_mode >> 6) & 0x03);
    }
    else if (dio == 1)
    {
      return ((_mode >> 4) & 0x03);
    }
    else if (dio == 2)
    {
      return ((_mode >> 2) & 0x03);
    }
    else if (dio == 3)
    {
      return (_mode & 0x03);
    }
  }
  else if (dio < 6)
  {
    int _mode = readReg(REG_DIO_MAPPING_2);
    ESP_LOGD(TAGLORA, "REG_DIO_MAPPING_2=0x%02x", _mode);
    if (dio == 4)
    {
      return ((_mode >> 6) & 0x03);
    }
    else if (dio == 5)
    {
      return ((_mode >> 4) & 0x03);
    }
  }
  return 0;
}

/**
 * Set bandwidth (bit rate)
 * @param sbw Signal bandwidth(0 to 9)
 */
void LoraService::setBandwidth(int sbw)
{
  if (sbw < 10)
  {
    writeReg(REG_MODEM_CONFIG_1, (readReg(REG_MODEM_CONFIG_1) & 0x0f) | (sbw << 4));
  }
}

/**
 * Get bandwidth (bit rate)
 * @param sbw Signal bandwidth(0 to 9)
 */
int LoraService::getBandwidth(void)
{
  // int bw;
  // bw = readReg(REG_MODEM_CONFIG_1) & 0xf0;
  // ESP_LOGD(TAGLORA, "bw=0x%02x", bw);
  // bw = bw >> 4;
  // return bw;
  return ((readReg(REG_MODEM_CONFIG_1) & 0xf0) >> 4);
}

/**
 * Set coding rate
 * @param denominator 5-8, Denominator for the coding rate 4/x
 */
void LoraService::setCodingRate(int denominator)
{
  if (denominator < 5)
    denominator = 5;
  else if (denominator > 8)
    denominator = 8;

  int cr = denominator - 4;
  writeReg(REG_MODEM_CONFIG_1, (readReg(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

/**
 * Get coding rate
 */
int LoraService::getCodingRate(void)
{
  return ((readReg(REG_MODEM_CONFIG_1) & 0x0E) >> 1);
}

/**
 * Set the size of preamble.
 * @param length Preamble length in symbols.
 */
void LoraService::setPreambleLength(long length)
{
  writeReg(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
  writeReg(REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}

/**
 * Get the size of preamble.
 */
long LoraService::getPreambleLength(void)
{
  long preamble;
  preamble = readReg(REG_PREAMBLE_MSB) << 8;
  preamble = preamble + readReg(REG_PREAMBLE_LSB);
  return preamble;
}

/**
 * Change radio sync word.
 * @param sw New sync word to use.
 */
void LoraService::setSyncWord(int sw)
{
  writeReg(REG_SYNC_WORD, sw);
}

/**
 * Enable appending/verifying packet CRC.
 */
void LoraService::setCrc(bool crc)
{
  if (crc)
    writeReg(REG_MODEM_CONFIG_2, readReg(REG_MODEM_CONFIG_2) | 0x04);
  else
    writeReg(REG_MODEM_CONFIG_2, readReg(REG_MODEM_CONFIG_2) & 0xfb);
}

/**
 * Perform hardware initialization.
 */
int LoraService::init(void)
{
  esp_err_t ret;

  ESP_LOGI(TAGLORA, "Initializing service ...");

  /*
   * Configure CPU hardware to communicate with the radio chip
   */
  gpio_reset_pin(config.gpioRST);
  gpio_set_direction(config.gpioRST, GPIO_MODE_OUTPUT);
  gpio_reset_pin(config.gpioCS);
  gpio_set_direction(config.gpioCS, GPIO_MODE_OUTPUT);
  gpio_set_level(config.gpioCS, 1);

  spi_bus_config_t bus = {
      .mosi_io_num = config.gpioMOSI,
      .miso_io_num = config.gpioMISO,
      .sclk_io_num = config.gpioSCL,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = 0};

  // ret = spi_bus_initialize(VSPI_HOST, &bus, 0);
  ret = spi_bus_initialize(config.spiHost, &bus, SPI_DMA_CH_AUTO);
  assert(ret == ESP_OK);

  spi_device_interface_config_t dev = {
      .mode = 0,
      .clock_speed_hz = 9000000,
      .spics_io_num = config.gpioCS,
      .flags = 0,
      .queue_size = 7,
      .pre_cb = NULL};
  // ret = spi_bus_add_device(VSPI_HOST, &dev, &spi);
  ret = spi_bus_add_device(config.spiHost, &dev, &spi);
  assert(ret == ESP_OK);

  /*
   * Perform hardware reset.
   */
  reset();

  /*
   * Check version.
   */
  uint8_t version;
  uint8_t i = 0;
  while (i++ < TIMEOUT_RESET)
  {
    version = readReg(REG_VERSION);
    ESP_LOGD(TAGLORA, "version=0x%02x", version);
    if (version == 0x12)
      break;
    vTaskDelay(2);
  }
  ESP_LOGD(TAGLORA, "i=%d, TIMEOUT_RESET=%d", i, TIMEOUT_RESET);
  if (i == TIMEOUT_RESET + 1)
    return 0; // Illegal version
  // assert(i < TIMEOUT_RESET + 1); // at the end of the loop above, the max value i can reach is TIMEOUT_RESET + 1

  /*
   * Default configuration.
   */
  sleep();
  writeReg(REG_FIFO_RX_BASE_ADDR, 0);
  writeReg(REG_FIFO_TX_BASE_ADDR, 0);
  writeReg(REG_LNA, readReg(REG_LNA) | 0x03);
  writeReg(REG_MODEM_CONFIG_3, 0x04);
  setTxPower(17);

  setFrequency(config.loraFrequency);
  setCrc(config.crc);

  setCodingRate(config.codingRate);
  setBandwidth(config.bandwith);
  setSpreadingFactor(config.spreadingFactor);

  idle();

  return 1;
}

/**
 * Send a packet.
 * @param buf Data to be sent
 * @param size Size of data.
 */
void LoraService::sendPacket(std::vector<uint8_t> buffer)
{
  ESP_LOGI(TAGLORA, "Sending packet: size %d", buffer.size());

  /*
   * Transfer data to radio.
   */
  idle();
  writeReg(REG_FIFO_ADDR_PTR, 0);

#if BUFFER_IO
  writeRegBuffer(REG_FIFO, buffer.data(), buffer.size());
#else
  for (int i = 0; i < size; i++)
    writeReg(REG_FIFO, *buf++);
#endif

  writeReg(REG_PAYLOAD_LENGTH, buffer.size());

  /*
   * Start transmission and wait for conclusion.
   */
  writeReg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
  int loop = 0;
  while (1)
  {
    int irq = readReg(REG_IRQ_FLAGS);
    ESP_LOGD(TAGLORA, "readReg=0x%x", irq);
    if ((irq & IRQ_TX_DONE_MASK) == IRQ_TX_DONE_MASK)
      break;
    loop++;
    if (loop == 10)
      break;
    vTaskDelay(2);
  }
  if (loop == 10)
  {
    send_packet_lost++;
    ESP_LOGE(TAGLORA, "lora_send_packet Fail");
  }
  writeReg(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
}

/**
 * Read a received packet.
 * @param buf Buffer for the data.
 * @param size Available size in buffer (bytes).
 * @return Number of bytes received (zero if no packet available).
 */
int LoraService::receivePacket(uint8_t *buf, int size)
{
  int len = 0;

  /*
   * Check interrupts.
   */
  int irq = readReg(REG_IRQ_FLAGS);
  writeReg(REG_IRQ_FLAGS, irq);
  if ((irq & IRQ_RX_DONE_MASK) == 0)
    return 0;
  if (irq & IRQ_PAYLOAD_CRC_ERROR_MASK)
    return 0;

  /*
   * Find packet size.
   */
  if (implicit)
    len = readReg(REG_PAYLOAD_LENGTH);
  else
    len = readReg(REG_RX_NB_BYTES);

  /*
   * Transfer data from radio.
   */
  idle();
  writeReg(REG_FIFO_ADDR_PTR, readReg(REG_FIFO_RX_CURRENT_ADDR));
  if (len > size)
    len = size;
#if BUFFER_IO
  readRegBuffer(REG_FIFO, buf, len);
#else
  for (int i = 0; i < len; i++)
    *buf++ = readReg(REG_FIFO);
#endif

  ESP_LOGI(TAGLORA, "Received packet. length: %d", len);

  return len;
}

/**
 * Returns non-zero if there is data to read (packet received).
 */
bool LoraService::hasPacket(void)
{
  return readReg(REG_IRQ_FLAGS) & IRQ_RX_DONE_MASK;
}

/**
 * Returns RegIrqFlags.
 */
int LoraService::getIrq(void)
{
  return (readReg(REG_IRQ_FLAGS));
}

/**
 * Return lost send packet count.
 */
int LoraService::getPacketLost(void)
{
  return (send_packet_lost);
}

/**
 * Return last packet's RSSI.
 */
int LoraService::getPacketRSSI(void)
{
  return (readReg(REG_PKT_RSSI_VALUE) - (frequency < 868E6 ? 164 : 157));
}

/**
 * Return last packet's SNR (signal to noise ratio).
 */
float LoraService::getPacketSNR(void)
{
  return ((int8_t)readReg(REG_PKT_SNR_VALUE)) * 0.25;
}

/**
 * Shutdown hardware.
 */
void LoraService::close(void)
{
  sleep();
  //   close(spi);  FIXME: end hardware features after lora_close
  //   close(__cs);
  //   close(__rst);
  //   spi = -1;
  //   __cs = -1;
  //   __rst = -1;
}

void LoraService::dumpRegisters(void)
{
  int i;
  printf("00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F\n");
  for (i = 0; i < 0x40; i++)
  {
    printf("%02X ", readReg(i));
    if ((i & 0x0f) == 0x0f)
      printf("\n");
  }
  printf("\n");
}

LoraService::LoraService(LoraServiceConfig _config) : config(_config)
{
}

LoraService::~LoraService()
{
}
