/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_Empty/AP_HAL_Empty_Private.h>

#include "HAL_ESP32_Class.h"
#include "Scheduler.h"
#include "I2CDevice.h"
#include "SPIDevice.h"
#include "UARTDriver.h"
#include "WiFiDriver.h"
#include "WiFiUdpDriver.h"
#include "RCInput.h"
#include "RCOutput.h"
#include "GPIO.h"
#include "Storage.h"
#include "AnalogIn.h"
#include "Util.h"
#if AP_SIM_ENABLED
#include <AP_HAL/SIMState.h>
#endif

static ESP32::UARTDriver cons(0);
#ifdef HAL_ESP32_WIFI
#if HAL_ESP32_WIFI == 1
static ESP32::WiFiDriver serial1Driver; //tcp, client should connect to 192.168.4.1 port 5760
#elif HAL_ESP32_WIFI == 2
static ESP32::WiFiUdpDriver serial1Driver; //udp
#else
static Empty::UARTDriver serial1Driver;
#endif
#else
static Empty::UARTDriver serial1Driver;
#endif
static ESP32::UARTDriver serial2Driver(2);
static ESP32::UARTDriver serial3Driver(1);
static Empty::UARTDriver serial4Driver;
static Empty::UARTDriver serial5Driver;
static Empty::UARTDriver serial6Driver;
static Empty::UARTDriver serial7Driver;
static Empty::UARTDriver serial8Driver;
static Empty::UARTDriver serial9Driver;

#if HAL_WITH_DSP
static Empty::DSP dspDriver;
#endif

static ESP32::I2CDeviceManager i2cDeviceManager;
#if defined(HAL_ESP32_SPI_BUSES)
static ESP32::SPIDeviceManager spiDeviceManager;
#else
static Empty::SPIDeviceManager spiDeviceManager;
#endif
#if AP_HAL_ANALOGIN_ENABLED
static ESP32::AnalogIn analogIn;
#else
static Empty::AnalogIn analogIn;
#endif
#ifdef HAL_USE_EMPTY_STORAGE
static Empty::Storage storageDriver;
#else
static ESP32::Storage storageDriver;
#endif
static ESP32::GPIO gpioDriver;
#if AP_SIM_ENABLED
static Empty::RCOutput rcoutDriver;
#else
static ESP32::RCOutput rcoutDriver;
#endif
static ESP32::RCInput rcinDriver;
static ESP32::Scheduler schedulerInstance;
static ESP32::Util utilInstance;
static Empty::OpticalFlow opticalFlowDriver;
static Empty::Flash flashDriver;

#if AP_SIM_ENABLED
static AP_HAL::SIMState xsimstate;
#endif

extern const AP_HAL::HAL& hal;

HAL_ESP32::HAL_ESP32() :
    AP_HAL::HAL(
        &cons, //Console/mavlink
        &serial1Driver, //Telem 1
        &serial2Driver, //Telem 2
        &serial3Driver, //GPS 1
        &serial4Driver, //GPS 2
        &serial5Driver, //Extra 1
        &serial6Driver, //Extra 2
        &serial7Driver, //Extra 3
        &serial8Driver, //Extra 4
        &serial9Driver, //Extra 5
        &i2cDeviceManager,
        &spiDeviceManager,
        nullptr,
        &analogIn,
        &storageDriver,
        &cons,
        &gpioDriver,
        &rcinDriver,
        &rcoutDriver,
        &schedulerInstance,
        &utilInstance,
        &opticalFlowDriver,
        &flashDriver,
#if AP_SIM_ENABLED
        &xsimstate,
#endif
#if HAL_WITH_DSP
        &dspDriver,
#endif
        nullptr
    )
{}

void HAL_ESP32::run(int argc, char * const argv[], Callbacks* callbacks) const
{
#if AP_SIM_ENABLED
    AP::sitl()->init();
#endif  // AP_SIM_ENABLED

#ifdef HAL_ESP32_GPIO_INIT_LIST
    /*
      Drive GPIOs to a fixed level early in boot, as requested by hwdef.

      This is needed on boards carrying several sensors that share one I2C
      address, where the spare ones must be held in reset. The M5Stack
      StampFly has two VL53L3CX ToF sensors which both default to 0x29, so
      without driving their XSHUT pins both appear at the same address and
      collide on the bus.

      In hwdef.dat:
        define HAL_ESP32_GPIO_INIT_LIST { {9, 0}, {7, 1} }
    */
    {
        static const struct { uint8_t pin; uint8_t level; } gpio_init[] = HAL_ESP32_GPIO_INIT_LIST;
        for (const auto &g : gpio_init) {
            hal.gpio->pinMode(g.pin, HAL_GPIO_OUTPUT);
            hal.gpio->write(g.pin, g.level);
        }
    }
#endif

#ifdef HAL_ESP32_I2C_ADDR_INIT_LIST
    /*
      Bring up sensors which share a default I2C address one at a time, giving
      each its own address before the next is released, so that they can all be
      used together.

      Each step raises a pin, waits for the part to boot, and writes one byte
      to a 16 bit register on the device at the address it currently answers
      on. A zero register skips the write, which is what the last part wants
      since it keeps the default address.

      In hwdef.dat:
        define HAL_ESP32_I2C_ADDR_INIT_LIST { {9, 0, 0x29, 0x0001, 0x2A}, {7, 0, 0x29, 0, 0} }
    */
    {
        static const struct {
            uint8_t pin;    // pin releasing this part from reset
            uint8_t bus;
            uint8_t addr;   // address it answers on once booted
            uint16_t reg;   // register holding its address, 0 to skip
            uint8_t val;    // address to move it to
        } steps[] = HAL_ESP32_I2C_ADDR_INIT_LIST;

        for (const auto &s : steps) {
            hal.gpio->pinMode(s.pin, HAL_GPIO_OUTPUT);
            hal.gpio->write(s.pin, 1);
            vTaskDelay(pdMS_TO_TICKS(100));
            if (s.reg == 0) {
                continue;
            }
            AP_HAL::I2CDevice *dev = hal.i2c_mgr->get_device_ptr(s.bus, s.addr);
            if (dev == nullptr) {
                continue;
            }
            const uint8_t msg[3] { uint8_t(s.reg >> 8), uint8_t(s.reg & 0xff), s.val };
            WITH_SEMAPHORE(dev->get_semaphore());
            dev->transfer(msg, sizeof(msg), nullptr, 0);
        }
    }
#endif

    ((ESP32::Scheduler *)hal.scheduler)->set_callbacks(callbacks);
    hal.scheduler->init();
}

void AP_HAL::init()
{
}

