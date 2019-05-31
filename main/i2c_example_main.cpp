#include <stdio.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "sdkconfig.h"

static const char *TAG = "i2c-master";

#define DATA_LENGTH 512    /*!< Data buffer length of test buffer */
#define RW_TEST_LENGTH 128 /*!< Data length for r/w test, [0,DATA_LENGTH] */
#define DELAY_TIME_BETWEEN_ITEMS_MS \
  10000 /*!< delay time between different test items */

#define I2C_SLAVE_SCL_IO GPIO_NUM_22
#define I2C_SLAVE_SDA_IO GPIO_NUM_21
#define I2C_SLAVE_NUM I2C_NUM_0
#define I2C_SLAVE_TX_BUF_LEN (2 * DATA_LENGTH)
#define I2C_SLAVE_RX_BUF_LEN (2 * DATA_LENGTH)

#define I2C_MASTER_SCL_IO GPIO_NUM_19
#define I2C_MASTER_SDA_IO GPIO_NUM_18
#define I2C_MASTER_NUM I2C_NUM_1
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */

#define ESP_SLAVE_ADDR 0x28
#define WRITE_BIT I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ   /*!< I2C master read */
#define ACK_CHECK_EN 0x1           /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0 /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0       /*!< I2C ack value */
#define NACK_VAL 0x1      /*!< I2C nack value */

extern "C" {
void app_main(void);
}

/**
 * @brief test code to read esp-i2c-slave
 *        We need to fill the buffer of esp slave device, then master can read
 * them out.
 *
 * _______________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | read n-1 bytes + ack | read 1 byte +
 * nack | stop |
 * --------|--------------------------|----------------------|--------------------|------|
 *
 */
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

/**
 * @brief Test code to write esp-i2c-slave
 *        Master device write data to slave(both esp32),
 *        the data will be stored in slave buffer.
 *        We can read them out from slave buffer.
 *
 * ___________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write n bytes + ack  | stop |
 * --------|---------------------------|----------------------|------|
 *
 */
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

/**
 * @brief test function to show buffer
 */
static void disp_buf(uint8_t *buf, int len) {
  int i;
  for (i = 0; i < len; i++) {
    printf("%02x ", buf[i]);
    if ((i + 1) % 16 == 0) {
      printf("\n");
    }
  }
  printf("\n");
}

static void i2c_test_task(void *arg) {
  int i = 0;
  int ret;
  uint32_t task_idx = (uint32_t)arg;

  uint8_t *data_wr = (uint8_t *)malloc(DATA_LENGTH);
  uint8_t *data_rd = (uint8_t *)malloc(DATA_LENGTH);
  while (1) {
    for (i = 0; i < DATA_LENGTH; i++) {
      data_rd[i] = 0x00;
    }
    i2c_reset_rx_fifo(I2C_SLAVE_NUM);
    ret = i2c_master_read_slave(I2C_MASTER_NUM, data_rd, RW_TEST_LENGTH);
    if (ret == ESP_ERR_TIMEOUT) {
      ESP_LOGE(TAG, "I2C Timeout");
    } else if (ret == ESP_OK) {
      printf("====TASK[%d] Master read %d ====\n", task_idx, RW_TEST_LENGTH);
      disp_buf(data_rd, RW_TEST_LENGTH);
    } else {
      ESP_LOGW(TAG,
               "TASK[%d] %s: Master read slave error, IO not connected...\n",
               task_idx, esp_err_to_name(ret));
    }
    vTaskDelay((DELAY_TIME_BETWEEN_ITEMS_MS) / portTICK_RATE_MS);
    //---------------------------------------------------
    for (i = 0; i < DATA_LENGTH; i++) {
      data_wr[i] = i + 10;
    }
    // we need to fill the slave buffer so that master can read later
    ret = i2c_master_write_slave(I2C_MASTER_NUM, data_wr, RW_TEST_LENGTH);
    if (ret == ESP_ERR_TIMEOUT) {
      ESP_LOGE(TAG, "I2C Timeout");
    } else if (ret == ESP_OK) {
      printf("----TASK[%d] Master write %d ----\n", task_idx, RW_TEST_LENGTH);
      disp_buf(data_wr, RW_TEST_LENGTH);

    } else {
      ESP_LOGW(TAG,
               "TASK[%d] %s: Master write slave error, IO not connected....\n",
               task_idx, esp_err_to_name(ret));
    }
    vTaskDelay((DELAY_TIME_BETWEEN_ITEMS_MS) / portTICK_RATE_MS);
  }
  // vSemaphoreDelete(print_mux);
  vTaskDelete(NULL);
}

void app_main() {
  ESP_LOGI(TAG, "MASTER--------------------------------");
  ESP_ERROR_CHECK(i2c_master_init());

  xTaskCreate(i2c_test_task, "master", 1024 * 2, (void *)0, 10, NULL);
}
