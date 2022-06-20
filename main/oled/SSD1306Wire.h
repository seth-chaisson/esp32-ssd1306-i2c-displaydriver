/**
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 by ThingPulse, Daniel Eichhorn
 * Copyright (c) 2018 by Fabrice Weinberg
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * ThingPulse invests considerable time and money to develop these open source libraries.
 * Please support us by buying our products (and not the clones) from
 * https://thingpulse.com
 *
 */

#ifndef SSD1306Wire_h
#define SSD1306Wire_h

#include "OLEDDisplay.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

#define DISPLAY_ADDRESS 0x3c
#define COMMAND_BYTE 0x80
#define DISPLAY_REST_PIN 16
#define I2C_MASTER_SCL_IO 15		/*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO 4			/*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM 0			/*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ 400000	/*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000

class Wire
{

	i2c_cmd_handle_t handle;

public:
	void beginTransmission(uint8_t _address)
	{
		esp_err_t err = ESP_OK;
		uint8_t buffer[I2C_TRANS_BUF_MINIMUM_SIZE] = {0};

		this->handle = i2c_cmd_link_create_static(buffer, sizeof(buffer));
		assert(this->handle != NULL);

		err = i2c_master_start(this->handle);
		if (err != ESP_OK)
		{
			endTransmission();
			return;
		}
		err = i2c_master_write_byte(this->handle, _address << 1 | I2C_MASTER_WRITE, true);
		if (err != ESP_OK)
		{
			endTransmission();
			return;
		}
	}
	void write(uint8_t data)
	{
		err = i2c_master_write(this->handle, data, 8, true);
		if (err != ESP_OK)
		{
			endTransmission();
			return;
		}
	}
	void endTransmission()
	{
		i2c_master_stop(this->handle);
		err = i2c_master_cmd_begin(0, this->handle, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

		i2c_cmd_link_delete_static(this->handle);
		return err;
	}
}

class SSD1306Wire : public OLEDDisplay
{
private:
	uint8_t _address;
	uint8_t _sda;
	uint8_t _scl;
	uint8_t _rst;
	bool _doI2cAutoInit = false;

public:
	SSD1306Wire(uint8_t _address, uint8_t _sda, uint8_t _scl, uint8_t _rst, OLEDDISPLAY_GEOMETRY g = GEOMETRY_128_64)
	{
		setGeometry(g);

		this->_address = _address;
		this->_sda = _sda;
		this->_scl = _scl;
		this->_rst = _rst;
	}

	/**
	 * @brief i2c master initialization
	 */
	static esp_err_t i2c_master_init(void)
	{
		int i2c_master_port = I2C_MASTER_NUM;

		i2c_config_t conf = {
			.mode = I2C_MODE_MASTER,
			.sda_io_num = I2C_MASTER_SDA_IO,
			.scl_io_num = I2C_MASTER_SCL_IO,
			.sda_pullup_en = GPIO_PULLUP_ENABLE,
			.scl_pullup_en = GPIO_PULLUP_ENABLE,
			.master.clk_speed = I2C_MASTER_FREQ_HZ,
		};

		i2c_param_config(i2c_master_port, &conf);

		return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
	}
	void reset_display()
	{
		// pinMode(_rst,OUTPUT);
		// digitalWrite(_rst, LOW);
		// delay(50);
		// digitalWrite(_rst, HIGH);

		gpio_reset_pin(DISPLAY_REST_PIN);
		/* Set the GPIO as a push/pull output */
		gpio_set_direction(DISPLAY_REST_PIN, GPIO_MODE_OUTPUT);
		gpio_set_level(DISPLAY_REST_PIN, 0);
		vTaskDelay(50 / portTICK_PERIOD_MS);
		gpio_set_level(DISPLAY_REST_PIN, 1);
		vTaskDelay(150 / portTICK_PERIOD_MS);
	}
	static esp_err_t display_write_byte(uint8_t reg_addr, uint8_t data)
	{
		int ret;
		uint8_t write_buf[2] = {reg_addr, data};

		ret = i2c_master_write_to_device(I2C_MASTER_NUM, DISPLAY_ADDRESS, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

		return ret;
	}

	void ESP32sendCommand(uint8_t cmd)
	{
		display_write_byte(COMMAND_BYTE, cmd);
	}

	bool connect()
	{
		reset_display();

		i2c_master_init();

		return true;
	}

	void display(void)
	{
		initI2cIfNeccesary();
		if (rotate_angle == ANGLE_0_DEGREE || rotate_angle == ANGLE_180_DEGREE)
		{
			const int x_offset = (128 - this->width()) / 2;
#ifdef OLEDDISPLAY_DOUBLE_BUFFER
			uint8_t minBoundY = UINT8_MAX;
			uint8_t maxBoundY = 0;

			uint8_t minBoundX = UINT8_MAX;
			uint8_t maxBoundX = 0;
			uint8_t x, y;

			// Calculate the Y bounding box of changes
			// and copy buffer[pos] to buffer_back[pos];
			for (y = 0; y < (this->height() / 8); y++)
			{
				for (x = 0; x < this->width(); x++)
				{
					uint16_t pos = x + y * this->width();
					if (buffer[pos] != buffer_back[pos])
					{
						minBoundY = _min(minBoundY, y);
						maxBoundY = _max(maxBoundY, y);
						minBoundX = _min(minBoundX, x);
						maxBoundX = _max(maxBoundX, x);
					}
					buffer_back[pos] = buffer[pos];
				}
				// yield();
			}

			// If the minBoundY wasn't updated
			// we can savely assume that buffer_back[pos] == buffer[pos]
			// holdes true for all values of pos

			if (minBoundY == UINT8_MAX)
				return;

			sendCommand(COLUMNADDR);
			sendCommand(x_offset + minBoundX);
			sendCommand(x_offset + maxBoundX);

			sendCommand(PAGEADDR);
			sendCommand(minBoundY);
			sendCommand(maxBoundY);

			byte k = 0;
			for (y = minBoundY; y <= maxBoundY; y++)
			{
				for (x = minBoundX; x <= maxBoundX; x++)
				{
					if (k == 0)
					{
						Wire.beginTransmission(_address);
						Wire.write(0x40);
					}

					Wire.write(buffer[x + y * this->width()]);
					k++;
					if (k == 16)
					{
						Wire.endTransmission();
						k = 0;
					}
				}
				// yield();
			}

			if (k != 0)
			{
				Wire.endTransmission();
			}
#else

			sendCommand(COLUMNADDR);
			sendCommand(x_offset);
			sendCommand(x_offset + (this->width() - 1));

			sendCommand(PAGEADDR);
			sendCommand(0x0);
			sendCommand((this->height() / 8) - 1);

			if (geometry == GEOMETRY_128_64)
			{
				sendCommand(0x7);
			}
			else if (geometry == GEOMETRY_128_32)
			{
				sendCommand(0x3);
			}

			for (uint16_t i = 0; i < displayBufferSize; i++)
			{
				Wire.beginTransmission(this->_address);
				Wire.write(0x40);
				for (uint8_t x = 0; x < 16; x++)
				{
					Wire.write(buffer[i]);
					i++;
				}
				i--;
				Wire.endTransmission();
			}
#endif
		}
		else
		{
			uint8_t buffer_rotate[displayBufferSize];
			memset(buffer_rotate, 0, displayBufferSize);
			uint8_t temp;
			for (uint16_t i = 0; i < this->width(); i++)
			{
				for (uint16_t j = 0; j < this->height(); j++)
				{
					temp = buffer[(j >> 3) * this->width() + i] >> (j & 7) & 0x01;
					buffer_rotate[(i >> 3) * this->height() + j] |= (temp << (i & 7));
				}
			}
#ifdef OLEDDISPLAY_DOUBLE_BUFFER
			uint8_t minBoundY = UINT8_MAX;
			uint8_t maxBoundY = 0;

			uint8_t minBoundX = UINT8_MAX;
			uint8_t maxBoundX = 0;
			uint8_t x, y;
			const int x_offset = (128 - this->height()) / 2;
			// Calculate the Y bounding box of changes
			// and copy buffer[pos] to buffer_back[pos];
			for (y = 0; y < (this->width() / 8); y++)
			{
				for (x = 0; x < this->height(); x++)
				{
					uint16_t pos = x + y * this->height();
					if (buffer_rotate[pos] != buffer_back[pos])
					{
						minBoundY = _min(minBoundY, y);
						maxBoundY = _max(maxBoundY, y);
						minBoundX = _min(minBoundX, x);
						maxBoundX = _max(maxBoundX, x);
					}
					buffer_back[pos] = buffer_rotate[pos];
				}
				// yield();
			}
			if (minBoundY == UINT8_MAX)
				return;

			sendCommand(COLUMNADDR);
			sendCommand(x_offset + minBoundX);
			sendCommand(x_offset + maxBoundX);

			sendCommand(PAGEADDR);
			sendCommand(minBoundY);
			sendCommand(maxBoundY);

			byte k = 0;
			for (y = minBoundY; y <= maxBoundY; y++)
			{
				for (x = minBoundX; x <= maxBoundX; x++)
				{
					if (k == 0)
					{
						Wire.beginTransmission(_address);
						Wire.write(0x40);
					}

					Wire.write(buffer_rotate[x + y * this->height()]);
					k++;
					if (k == 16)
					{
						Wire.endTransmission();
						k = 0;
					}
				}
				// yield();
			}

			if (k != 0)
			{
				Wire.endTransmission();
			}
#else
			sendCommand(COLUMNADDR);
			sendCommand(x_offset);
			sendCommand(x_offset + (this->height() - 1));

			sendCommand(PAGEADDR);
			sendCommand(0x0);
			sendCommand((this->width() / 8) - 1);
			if (geometry == GEOMETRY_128_64)
			{
				sendCommand(0x7);
			}
			else if (geometry == GEOMETRY_128_32)
			{
				sendCommand(0x3);
			}

			for (uint16_t i = 0; i < displayBufferSize; i++)
			{
				Wire.beginTransmission(this->_address);
				Wire.write(0x40);
				for (uint8_t x = 0; x < 16; x++)
				{
					Wire.write(buffer_rotate[i]);
					i++;
				}
				i--;
				Wire.endTransmission();
			}
#endif
		}
	}

	void setI2cAutoInit(bool doI2cAutoInit)
	{
		_doI2cAutoInit = doI2cAutoInit;
	}

private:
	int getBufferOffset(void)
	{
		return 0;
	}
	inline void sendCommand(uint8_t command) __attribute__((always_inline))
	{
		initI2cIfNeccesary();
		ESP32sendCommand()
	}

	void initI2cIfNeccesary()
	{
		if (_doI2cAutoInit)
		{
			reset_display();
			i2c_master_init();
		}
	}
};

#endif
