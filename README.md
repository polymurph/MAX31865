# MAX31865

This is a driver for the [MAX31865](https://www.maximintegrated.com/en/products/interface/sensor-interface/MAX31865.html) RTD-to-Digital converter (RTD: resistance temperature detectors).



## Getting Started

These instructions will help you implement the driver step by step into your project.

The driver has an object-oriented approach which enbales a certain abstraction which helps to minimize complexety and improve readability.

### Prerequisites
* [Download](https://github.com/polymurph/MAX31865/archive/master.zip) and place the max31865.c/.h files into a desired folder inside your project.
* Knowledge about the functionality of the Device and it's hardware aspects (see [Datasheet](https://datasheets.maximintegrated.com/en/ds/MAX31865.pdf)).

### Creating an Object

Firts we have to include the driver as following...
```c
#include "<your path>/max31865.h"
```
Then we can then create an instance...

```c
max31865_t TempSensor;
```
This instance has not yet been initialized and must be done with the init function to complete the process as following...
```c
max31865_init(&TempSensor,...);
```
The init function will update all the struct members of TempSensor with your desired settings. It will automatically write the desired configurations and the upper and lower temperature fault thresholds directly to the device (MAX31865).

#### Init Parameters
```c
max31865_init(max31865_t*  device,...);
```
**device** is a pointer to your Object. Your Oject must be passed by pointer to be able to initialized.
```c
max31865_init(&TempSensor,...);
```
-----

```c
max31865_init(...,fptr_b_t chipselect_cb,...);
```
**chipselect_cp** is a function pointer to a chipselect function for SPI interface. This function shold contain the needed GPIO manipulations to select the device. The callback function must have a bool type as parameter and void type as return parameter as shown here...

```c
void chipselect(boot enable) {
  // GPIO manipulations
}
```

-----

```c
max31865_init(...,u8_fptr_u8_t spi_trx_cb,,...);
```
**spi_trx_cb** is a function pointer to a callback for a SPI transrecive (full duplex) function. This function should contain all the Master SPI transrecive manipulations.The callback function must have a uint8_t type as parameter and uint8_t type as return parameter as shown here... 
```c
uint8_t spi_trx(uint8_t data) {
  // SPI manipulations
}
```

----

```c
max31865_init(..., fptr_t charged_time_delay_cb,...);
```
**charged_time_delay_cb** is a function pointer to a callback for the chargetime delay function. This function should contain a delay cycle for at leas 5 times the charging constant given by the capacitance of the capacitor between RTDI+ and RTDIN- and resistance of the RTD. One possibility is to use a delay from a RTOS system.

![](https://raw.githubusercontent.com/polymurph/MAX31865/master/t_charge.png)

The callback function must have a void type as parameter and void type as return parameter as shown here...
```c
void charge_time_delay_cb(void) {
  // 5*tau delay
}
```

----

```c
max31865_init(..., fptr_t conversion_timer_deay_cb,...);
```
**conversion_timer_deay_cb** is a function pointer to a callback for the conversion time delay function.


The callback function must have a void type as parameter and void type as return parameter as shown here...
```c
void charge_time_delay_cb(void) {
  // 5*tau delay
}
```








 

### Measuring Temperature


## Contributing

Please read [CONTRIBUTING.md]() for details on my code of conduct, and the process for submitting pull requests to us.

## Authors

* **Edwin Koch** - *Initial work* - [polymurph](https://github.com/polymurph)

See also the list of [contributors](https://github.com/polymurph/MAX31865/graphs/contributors) who participated in this project.

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments
* 
