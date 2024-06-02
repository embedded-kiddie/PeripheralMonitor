# PeripheralMonitor
For the purpose of learning the [RA4M1], this program monitors peripheral registers through the Arduino Minima/WiFe external serial I/F at `D0(RX)` / `D1(TX)`.

## How to use

1. Connect to your host PC with Minima/WiFi board via usb serial converter module.

![Connection between Arduino UNO R4 Minima/WiFi bord and your host PC][1]

2. Run the VT100 terminal emulator software such as [Tera Term][2] on your Windows PC. The configuration of serial communication protocol parameters such as the data, parity, and stop bits should be same as [`SERIAL_8N1`][3].  <br>  
  In case of Mac, please follow the instructions below:  
  - Run the Termina.app
  - Type `ls -l /dev/tty.*` to find the USB serial device file.
  - Type `screen /dev/tty.usbserialxxxxx 230400`, where `/dev/tty.usbserialxxxxx` is the name of your USB serial device file and, `230400` is baud rate.
  - To quit the `screen` command, type `CTRL-a` + `k`.
  - To reset your Terminal.app, type `reset`.

3. Download the latest version of this program from [latest release], and install it from **Sketch** → **Include Library** → **Add .ZIP Library…** on Arduino IDE.

4. Open the example sketch from **File** → **Examples** → **PeripheralMonitor** → **monitor_by_rtc** in _Examples from Custom Libraries_.

5. Click `Upload` button to compile, upload and execute the program.

## Screens

You can see the following five screens through the terminal emulator on your host PC.

- PORTS  
  Port Control register from PORT1 to PORT4  
  ![PORTS]
- PORT _n_  
  Port Control register of PORT _n_ where _n_ is `0` to `9`, such as `PORT1`
  ![PORT1]
- P _mn_  
  Port Function Select register of P _mn_ where _m_ is `0` to `9` and _n_ is `00` to `15`, such as `P102`
  ![P102]
- PINS  
  Port Function Select register from `D0` to `D19` of Arduino pins
  ![PINS]
- AGT _n_
  Asynchronous General-Purpose Timer register of AGT _n_ where _n_ is `0` or `1`, such as `AGT0`
  ![AGT0]

## Public methods in `PeripheralMonitor` class

### `begin`

```c++
bool begin(int baud_rate, PeripheralType_t type = PERIPHERAL_PORTS, int arg = 0)
```

This method initializes the `Serial1` object and sets the initial screen.  
  - `int baud_rate`  
  It must match the settings in the terminal emulator.
  - `PeripheralType_t type`  
  To set the initial screen, specify one of the following enumeration numbers:

```c++
/*----------------------------------------------------------------------
 * Peripheral types
 *----------------------------------------------------------------------*/
typedef enum {
  PERIPHERAL_PORTS, // PORT0 〜 PORT9 (default)
  PERIPHERAL_PORT,  // PORT0 〜 PORT9
  PERIPHERAL_PFS,   // PmnPFS (P000 〜 P915)
  PERIPHERAL_PINS,  // D0 〜 D19 (A0 〜 A5)
  PERIPHERAL_AGT,   // AGT0 〜 AGT1
} PeripheralType_t;
```
  - `int arg`  
  Specify a numerical value according to `PeripheralType_t`.

### `setup_register`

```c++
void setup_register(PeripheralType_t _type = PERIPHERAL_PORTS, int _arg = 0)
```
This method, like the method `begin`, changes the type of a peripheral.

### `show_caption`

```c++
void show_caption(void)
```

This method sends to the table caption to your host PC.

### `show_register`

```c++
void show_register(void)
```

This method dump the registers into your host PC.

### `scan_command`

```c++
void scan_command(void)
```

This method accepts your commands from Arduino IDE Serial Monitor.

You can type `?` or `help` into the Serial Monitor to see the following result:

```bash
Possible commands: ports, port, pins, p, agt, a, d, ?, help
```

For example, you can type such as `port1`, `p102`, `agt0`, `a0` and `d13`.

When you type only return key without any commands, `show_caption` method is executed.

## Sample sketch

See [monitor_by_rtc.ino][4] using RTC for periodic monitoring in the [_examples_][5] directory.


[1]:https://embedded-kiddie.github.io/images/2024/06-03/fritzig.jpg

[2]:https://teratermproject.github.io/index-en.html "Tera Term Open Source Project"

[3]:https://www.arduino.cc/reference/en/language/functions/communication/serial/begin/ "Serial.begin() - Arduino Reference"

[4]:https://github.com/embedded-kiddie/PeripheralMonitor/blob/main/examples/monitor_by_rtc/monitor_by_rtc.ino "PeripheralMonitor/examples/monitor_by_rtc/monitor_by_rtc.ino at main · embedded-kiddie/PeripheralMonitor"

[5]:https://github.com/embedded-kiddie/PeripheralMonitor/tree/main/examples/monitor_by_rtc "PeripheralMonitor/examples/monitor_by_rtc at main · embedded-kiddie/PeripheralMonitor"

[RA4M1]:https://www.renesas.com/us/en/products/microcontrollers-microprocessors/ra-cortex-m-mcus/ra4m1-32-bit-microcontrollers-48mhz-arm-cortex-m4-and-lcd-controller-and-cap-touch-hmi ""

[latest release]:https://github.com/embedded-kiddie/PeripheralMonitor/releases/tag/v0.1.0 "Release First release. · embedded-kiddie/PeripheralMonitor"

[PORTS]:https://embedded-kiddie.github.io/images/2024/06-03/registers-PORTS.gif

[PORT1]:https://embedded-kiddie.github.io/images/2024/06-03/registers-PORT1.gif

[P102]:https://embedded-kiddie.github.io/images/2024/06-03/registers-P102.gif

[PINS]:https://embedded-kiddie.github.io/images/2024/06-03/registers-PINS.gif

[AGT0]:https://embedded-kiddie.github.io/images/2024/06-03/registers-AGT.gif
