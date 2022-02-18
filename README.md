HX711 driver (based on WiringPi) for Raspberry Pi

# HX711CPP

## Dependencies

Program depends on WiringPi library, which can be installed using the following command:
```sh
sudo apt intall wiringpi
```

Of course, you must have g++ and make tools.

## Build

Build process is pretty simple:
```sh
CFLAGS=-O2 make -j4
```

You'll get `hx711` executable file.

## Run

Format:
```sh
./hx711 <human_mode> <correction_factor> <offset> <alignment_string> <moving_average> <times> <dout> <sck> <deviation_factor> <deviation_value> <retries> <use_ta_filter> <use_kalman_filter> <kalman_q> <kalman_r> <kalman_f> <kalman_h> <temperature_filename> <temperature_factor> <base_temperature> <debug>
```

* **int** _human mode_ - 0 - Normal mode, 1 - Human mode (input and output all values as decimal except alignment string)
* **double** (Human mode) **string** _correction factor_ - correction factor, multiplies to a result value
* **int** _offset_ - result offset, appends to a result value
* **char** <strong>*</strong> _alignment_string_ - ascii-coded 16 bytes of `double` `k` and `b` factors from `y = k * x + b`
* **unsigned int** _moving average_ - moving average size
* **unsigned int** _times_ - count of consecutive measurements, which are used to reduce the value volatility
* **unsigned int** _dout_ - _data out_ pin number (BCM)
* **unsigned int** _sck_ - _serial clock_ pin number (BCM)
* **int** _deviation factor_ - tolerance percentage
* **int** _deviation value_ - tolerance
* **unsigned int** _retries_ - count of retries before agree invalid values
* **int** _use TA filter_ - 0 - disable Tulpa Automatics filter, 1 - enable
* **int** _use Kalman filter_ - 0 - disable Kalman filter, 1 - enable
* **double** (Human mode) **string** _Kalman Q_ - the covariance of the process noise
* **double** (Human mode) **string** _Kalman R_ - the covariance of the observation noise
* **double** (Human mode) **string** _Kalman F_ - the state-transition model (set to `1`)
* **double** (Human mode) **string** _Kalman H_ - the observation model (set to `1`)
* **string** _temperature filename_ - a name of file contains temperature value
* **double** (Human mode) **string** _temperature factor_ - temperature compensation factor 
* **int** _base temperature_ - reference temperature value (in thousandths of degrees Celsius)
* **int** _debug_ - 0 - disable debug, 1 - enable (debug messages outputs to stderr)

In Normal mode program writes an ascii-coded `double` values to `stdout`.

## License

[LICENSE](./LICENSE) LGPLv3.