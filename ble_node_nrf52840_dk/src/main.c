/*
 * ADXL345 가속도계 (GY-291) → 1602 LCD (PCF8574 I2C 백팩) 출력
 *
 * I2C 버스 (P0.26 SDA / P0.27 SCL) 공유:
 *   ADXL345  @ 0x53
 *   PCF8574  @ 0x27  (A0=A1=A2=1 기본값)
 *
 * PCF8574 비트 배치 (표준 I2C 백팩):
 *   P7-P4 = D7-D4 (LCD 데이터 상위 니블)
 *   P3    = BL (백라이트)
 *   P2    = E  (Enable)
 *   P1    = RW (항상 0 = Write)
 *   P0    = RS (0=Command, 1=Data)
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <stdio.h>

#define ACCEL_NODE       DT_NODELABEL(adxl345)
#define LCD_I2C_NODE     DT_NODELABEL(i2c0)
#define LCD_ADDR         0x27
#define POLL_INTERVAL_MS 500

/* PCF8574 핀 마스크 */
#define LCD_RS  BIT(0)
#define LCD_RW  BIT(1)
#define LCD_E   BIT(2)
#define LCD_BL  BIT(3)

static const struct device *lcd_i2c;

/* ── PCF8574 / HD44780 저수준 ──────────────────────────────────────── */

static void pcf_write(uint8_t val)
{
	i2c_write(lcd_i2c, &val, 1, LCD_ADDR);
}

static void lcd_pulse_enable(uint8_t data)
{
	pcf_write(data | LCD_E);
	k_busy_wait(1);
	pcf_write(data & ~LCD_E);
	k_busy_wait(50);
}

static void lcd_write_nibble(uint8_t nibble, uint8_t flags)
{
	uint8_t data = (nibble << 4) | flags | LCD_BL;
	lcd_pulse_enable(data);
}

static void lcd_send(uint8_t byte, uint8_t flags)
{
	lcd_write_nibble(byte >> 4, flags);
	lcd_write_nibble(byte & 0x0F, flags);
	k_busy_wait(50);
}

static inline void lcd_cmd(uint8_t cmd) { lcd_send(cmd, 0); }
static inline void lcd_putc(char c)     { lcd_send((uint8_t)c, LCD_RS); }

/* ── LCD 초기화 (4비트 모드) ───────────────────────────────────────── */

static void lcd_init(void)
{
	k_msleep(50);

	/* 4비트 초기화 시퀀스 */
	lcd_write_nibble(0x03, 0); k_msleep(5);
	lcd_write_nibble(0x03, 0); k_busy_wait(150);
	lcd_write_nibble(0x03, 0); k_busy_wait(150);
	lcd_write_nibble(0x02, 0); k_busy_wait(150);

	lcd_cmd(0x28); /* 4비트, 2라인, 5×8 */
	lcd_cmd(0x0C); /* Display ON, 커서 OFF */
	lcd_cmd(0x06); /* 자동 증가, 시프트 없음 */
	lcd_cmd(0x01); /* 화면 지우기 */
	k_msleep(2);
}

static void lcd_set_cursor(uint8_t col, uint8_t row)
{
	uint8_t addr = col + (row == 0 ? 0x00 : 0x40);
	lcd_cmd(0x80 | addr);
}

static void lcd_puts(const char *s)
{
	while (*s) {
		lcd_putc(*s++);
	}
}

/* ── 센서값 포맷 (6문자: 부호 + 2정수 + '.' + 2소수) ─────────────── */

static void fmt_sv(char *buf, const struct sensor_value *sv)
{
	int32_t i = sv->val1;
	int32_t f = sv->val2;

	/* 부호 통일 */
	bool neg = (i < 0) || (i == 0 && f < 0);
	if (i < 0) { i = -i; }
	if (f < 0) { f = -f; }

	/* 소수점 2자리 (micro → centi) */
	f = f / 10000;

	snprintf(buf, 7, "%c%02d.%02d", neg ? '-' : ' ', (int)i, (int)f);
}

/* ── 메인 ──────────────────────────────────────────────────────────── */

int main(void)
{
	const struct device *accel = DEVICE_DT_GET(ACCEL_NODE);

	lcd_i2c = DEVICE_DT_GET(LCD_I2C_NODE);

	if (!device_is_ready(lcd_i2c)) {
		printk("I2C bus not ready\n");
		return -ENODEV;
	}

	if (!device_is_ready(accel)) {
		printk("ADXL345 not ready\n");
		return -ENODEV;
	}

	lcd_init();
	lcd_set_cursor(0, 0);
	lcd_puts("  ADXL345 Init  ");
	lcd_set_cursor(0, 1);
	lcd_puts("  Waiting...    ");
	k_msleep(1000);

	while (1) {
		struct sensor_value ax, ay, az;
		char xbuf[7], ybuf[7], zbuf[7];
		char line[17];

		if (sensor_sample_fetch(accel) < 0) {
			lcd_set_cursor(0, 0);
			lcd_puts("  Fetch Error   ");
			k_msleep(POLL_INTERVAL_MS);
			continue;
		}

		sensor_channel_get(accel, SENSOR_CHAN_ACCEL_X, &ax);
		sensor_channel_get(accel, SENSOR_CHAN_ACCEL_Y, &ay);
		sensor_channel_get(accel, SENSOR_CHAN_ACCEL_Z, &az);

		fmt_sv(xbuf, &ax);
		fmt_sv(ybuf, &ay);
		fmt_sv(zbuf, &az);

		/* Line 0: "X=-01.23Y= 09.81" → 16자 */
		snprintf(line, sizeof(line), "X=%sY=%s", xbuf, ybuf);
		lcd_set_cursor(0, 0);
		lcd_puts(line);

		/* Line 1: "Z=-01.23 m/s2   " → 16자 */
		snprintf(line, sizeof(line), "Z=%s m/s2   ", zbuf);
		lcd_set_cursor(0, 1);
		lcd_puts(line);

		k_msleep(POLL_INTERVAL_MS);
	}

	return 0;
}
