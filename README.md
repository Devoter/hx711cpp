Simple hx711 driver (based on WiringPi) for Raspberry Pi

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
make -j4
```

You'll get `hx711` executable file.

## Run

Format:
```sh
./hx711 <offset> <alignment_string> <moving_average> <times> <dout> <sck>
```

* **double** _offset_ - result offset, appends to a result value
* **char** <strong>*</strong> _alignment_string_ - ascii-coded 16 bytes of `double` `k` and `b` factors from `y = k * x + b`
* **unsigned int** _times_ - count of consecutive measurements, which are used to reduce the value volatility
* **unsigned int** _dout_ - _data out_ pin number (BCM)
* **unsigned int** _sck_ - _serial clock_ pin number (BCM)

Program writes an ascii-coded `double` values to `stdout`.
