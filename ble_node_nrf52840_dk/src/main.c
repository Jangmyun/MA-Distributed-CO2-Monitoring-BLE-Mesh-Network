/*
 * 1602 LCD (PCF8574 I2C 백팩) 테스트
 *
 * 연결:
 *   GND → GND  (J2 핀1)
 *   VCC → 3.3V (J2 핀2) 또는 5V (P22 핀3)
 *   SDA → P0.26 (J3 핀7)
 *   SCL → P0.27 (J3 핀8)
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include "lcd1602.h"

LOG_MODULE_REGISTER(lcd_test, LOG_LEVEL_DBG);

#define I2C_NODE    DT_NODELABEL(i2c0)
#define LCD_ADDR    LCD1602_DEFAULT_ADDR  /* 0x27 — 주소 다르면 0x3F 시도 */

int main(void)
{
	const struct device *i2c_dev = DEVICE_DT_GET(I2C_NODE);
	int ret;
	int count = 0;

	if (!device_is_ready(i2c_dev)) {
		LOG_ERR("I2C device not ready");
		return -ENODEV;
	}
	LOG_INF("I2C device ready: %s", i2c_dev->name);

	/* I2C 스캔: 0x20~0x27 범위에서 PCF8574 탐지 */
	LOG_INF("Scanning I2C bus...");
	for (uint8_t a = 0x20; a <= 0x3F; a++) {
		uint8_t dummy;
		if (i2c_read(i2c_dev, &dummy, 1, a) == 0) {
			LOG_INF("  Found device at 0x%02X", a);
		}
	}

	ret = lcd1602_init(i2c_dev, LCD_ADDR);
	if (ret) {
		LOG_ERR("LCD init failed (err %d) — 주소/배선 확인", ret);
		return ret;
	}
	LOG_INF("LCD 1602 initialized");

	/* 1행: 고정 타이틀 */
	lcd1602_set_cursor(i2c_dev, LCD_ADDR, 0, 0);
	lcd1602_write_str(i2c_dev, LCD_ADDR, "nRF52840 LCD OK!");

	while (1) {
		char buf[17];

		/* 2행: 카운터 갱신 */
		snprintf(buf, sizeof(buf), "Count: %-9d", count++);
		lcd1602_set_cursor(i2c_dev, LCD_ADDR, 0, 1);
		lcd1602_write_str(i2c_dev, LCD_ADDR, buf);

		LOG_DBG("LCD update: %s", buf);
		k_msleep(1000);
	}

	return 0;
}
