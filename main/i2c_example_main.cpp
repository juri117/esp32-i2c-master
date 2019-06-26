#include <stdio.h>
#include <cstring>
#include "driver/i2c.h"
#include "esp_log.h"
#include "sdkconfig.h"

static const char *TAG = "i2c-master";

#define DELAY_TIME_BETWEEN_ITEMS_MS \
  5000 /*!< delay time between different test items */

#define I2C_MASTER_SDA_IO GPIO_NUM_18
#define I2C_MASTER_SCL_IO GPIO_NUM_19

#define I2C_MASTER_NUM I2C_NUM_1
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */

#define ESP_SLAVE_ADDR 0x04
#define WRITE_BIT I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ   /*!< I2C master read */
#define ACK_CHECK_EN 0x1           /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0 /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0       /*!< I2C ack value */
#define NACK_VAL 0x1      /*!< I2C nack value */

#define SLAVE_REQUEST_WAIT_MS 80

const uint8_t testCmd[10] = {0x01, 0x02, 0x03, 0x04, 0x05,
                             0x06, 0x07, 0x08, 0x09, 0x0A};

extern "C" {
void app_main(void);
}

esp_err_t i2c_probe(i2c_port_t port, uint8_t address) {
  esp_err_t result;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  result = i2c_master_cmd_begin(port, cmd, 10 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  return result;
}

void i2c_scan(i2c_port_t port) {
  ESP_LOGD(TAG, "Scanning I2C bus.");

  uint8_t address;
  esp_err_t result;
  printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
  printf("00:         ");
  for (address = 3; address < 0x78; address++) {
    result = i2c_probe(port, address);

    if (address % 16 == 0) {
      printf("\n%.2x:", address);
    }
    if (result == ESP_OK) {
      printf(" %.2x", address);
    } else {
      printf(" --");
    }
  }
  printf("\n");
}

static esp_err_t i2c_master_read_slave(i2c_port_t i2c_num, uint8_t *data_rd,
                                       size_t size) {
  if (size == 0) {
    return ESP_OK;
  }
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
  if (size > 1) {
    i2c_master_read(cmd, data_rd, size - 1, (i2c_ack_type_t)ACK_VAL);
  }
  i2c_master_read_byte(cmd, data_rd + size - 1, (i2c_ack_type_t)NACK_VAL);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return ret;
}

static esp_err_t i2c_master_write_slave(i2c_port_t i2c_num, uint8_t *data_wr,
                                        size_t size) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return ret;
}

int read_register(i2c_port_t i2c_num, uint16_t address, uint8_t regAdd,
                  uint8_t *readBuff, uint16_t readBuffLen) {
  if (readBuffLen == 0) {
    return ESP_OK;
  }
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  // first, send device address (indicating write) & register to be read
  i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
  // send register we want
  i2c_master_write_byte(cmd, regAdd, ACK_CHECK_EN);
  // Send repeated start
  i2c_master_start(cmd);
  // now send device address (indicating read) & read data
  i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
  if (readBuffLen > 1) {
    i2c_master_read(cmd, readBuff, readBuffLen - 1, (i2c_ack_type_t)ACK_VAL);
  }
  i2c_master_read_byte(cmd, readBuff + readBuffLen - 1,
                       (i2c_ack_type_t)NACK_VAL);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return ret;
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init() {
  i2c_port_t i2c_master_port = I2C_MASTER_NUM;
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = I2C_MASTER_SDA_IO;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_io_num = I2C_MASTER_SCL_IO;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
  i2c_param_config(i2c_master_port, &conf);
  return i2c_driver_install(i2c_master_port, conf.mode,
                            I2C_MASTER_RX_BUF_DISABLE,
                            I2C_MASTER_TX_BUF_DISABLE, 0);
}

uint16_t slave_data_avlble() {
  uint8_t lenBuff[2];
  // int ret = read_register(I2C_MASTER_NUM, ESP_SLAVE_ADDR, 0x01, lenBuff, 2);
  uint8_t cmdBuff[1] = {0x01};
  int ret = i2c_master_write_slave(I2C_MASTER_NUM, cmdBuff, 1);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "error, slave_data_avlble write: %d", ret);
    return 0;
  }
  vTaskDelay(pdMS_TO_TICKS(SLAVE_REQUEST_WAIT_MS));
  ret = i2c_master_read_slave(I2C_MASTER_NUM, lenBuff, 2);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "error, slave_data_avlble read: %d", ret);
    return 0;
  }
  // ESP_LOG_BUFFER_HEX(TAG, lenBuff, 2);
  uint16_t len = lenBuff[0] | (lenBuff[1] << 8);
  ESP_LOGI(TAG, "data available: %d, ret: %d", len, ret);
  return len;
}

int slave_read_data(uint8_t *buff, uint16_t len) {
  uint8_t cmdBuff[1] = {0x02};
  int ret = i2c_master_write_slave(I2C_MASTER_NUM, cmdBuff, 1);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "error, slave_read_data write: %d", ret);
    return 0;
  }
  vTaskDelay(pdMS_TO_TICKS(SLAVE_REQUEST_WAIT_MS));
  ret = i2c_master_read_slave(I2C_MASTER_NUM, buff, len);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "error, slave_read_data read: %d", ret);
    return 0;
  }
  // int ret = read_register(I2C_MASTER_NUM, ESP_SLAVE_ADDR, 0x02, buff, len);
  return ret;
}

static void i2c_test_task(void *arg) {
  uint8_t inBuff[32];
  uint16_t inLen;
  while (1) {
    inLen = slave_data_avlble();
    if (inLen > 0) {
      if (inLen > 32) {
        ESP_LOGE(TAG, "in len too big: %d", inLen);
      } else {
        slave_read_data(inBuff, inLen);
        ESP_LOGI(TAG, "got data: %d", inLen);
        ESP_LOG_BUFFER_HEX(TAG, inBuff, inLen);
      }
    }
    vTaskDelay((DELAY_TIME_BETWEEN_ITEMS_MS) / portTICK_RATE_MS);
  }
  vTaskDelete(NULL);
}

static void i2c_sender(void *arg) {
  uint8_t outBuff[64];
  for (size_t i = 0; i < 64; i++) {
    outBuff[i] = i + 1;
  }

  while (1) {
    // memcpy(outBuff, testCmd, 10);
    i2c_master_write_slave(I2C_MASTER_NUM, outBuff, 64);
    vTaskDelay((2 * DELAY_TIME_BETWEEN_ITEMS_MS) / portTICK_RATE_MS);
  }
  vTaskDelete(NULL);
}

void app_main() {
  ESP_LOGI(TAG, "MASTER--------------------------------");
  ESP_ERROR_CHECK(i2c_master_init());

  i2c_scan(I2C_MASTER_NUM);

  xTaskCreate(i2c_test_task, "master", 1024 * 2, (void *)0, 10, NULL);
  vTaskDelay(pdMS_TO_TICKS(2500));
  xTaskCreate(i2c_sender, "sender", 1024 * 2, (void *)0, 10, NULL);
}
