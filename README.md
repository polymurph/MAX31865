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
----------------------------------------------------

```c
max31865_init(...,fptr_b_t chipselect_cb,...);
```
**chipselect_cp** is a function pointer to a chipselect function for SPI interface. This function shold contain the needed GPIO manipulations to select the device. The callback function must have a bool type as parameter and void type as return parameter as shown here...

```c
void chipselect(boot enable) {
  // GPIO manipulations
}
```

----------------------------------------------------

```c
max31865_init(...,u8_fptr_u8_t spi_trx_cb,,...);
```
**spi_trx_cb** is a function pointer to a callback for a SPI transrecive (full duplex) function. This function should contain all the Master SPI transrecive manipulations.The callback function must have a uint8_t type as parameter and uint8_t type as return parameter as shown here... 
```c
uint8_t spi_trx(uint8_t data) {
  // SPI manipulations
}
```

----------------------------------------------------

```c
max31865_init(..., fptr_t charged_time_delay_cb,...);
```
**charged_time_delay_cb** is a function pointer to a callback for the chargetime delay function. This function should contain a delay cycle for at leas 10.5 times the charging constant given by the capacitance of the capacitor between RTDI+ and RTDIN- and resistance of the RTD. One possibility is to use a delay from a RTOS system.

![](https://raw.githubusercontent.com/polymurph/MAX31865/master/formula_charge_time.png)

The callback function must have a void type as parameter and void type as return parameter as shown here...
```c
void charge_time_delay_cb(void) {
  // 10.5*tau delay
}
```

----------------------------------------------------

```c
max31865_init(..., fptr_t conversion_timer_deay_cb,...);
```
**conversion_timer_deay_cb** is a function pointer to a callback for the conversion time delay function. The delay time is dependent on the choice of the common mode filter. The callback function must have a void type as parameter and void type as return parameter as shown here...
```c
void conversion_time_delay_cb(void) {
  // 65.2 ms for 50Hz or 52 ms for 60Hz surpression
}
```
----------------------------------------------------

```c
max31865_init(..., fptr_t highFaultThreshold_callback, 
                   fptr_t lowFaultThreshold_callback, ...);
```
**highFaultThreshold_callback** and **lowFaultThreshold_callback** are both function pointers to callbacks to handle a high and low threshold fault.
The callback function must have a void type as parameter and void type as return parameter as shown here...
```c
void threshold_fault(void) {
  // handle threshold fault
}
```

----------------------------------------------------

```c
max31865_init(..., uint16_t rtd_ohm,...);
```
**rtd_ohm** is the value of the RTD in Ohms at 0°C. For example; a PT100 has a resistiv value of 100 Ohms at 0°C.

----------------------------------------------------

```c
max31865_init(..., uint16_t rref_ohm,...);
```
**rref_ohm** is the value of the Reference Resistor.

----------------------------------------------------

```c
max31865_init(..., uint16_t lowerFaulThreshold,
                   uint16_t higherFaultThreshold, ...);
```
**lowerFaulThreshold** and **higherFaultThreshold** are the threshold values. The ADC has a 15 bit ADC which implies a range from 0x0000 to 0x7FFF for the threshold values. Note that the upper threshold value must be greater than the lower one.

----------------------------------------------------

```c
max31865_init(..., bool wire_3,...);
```
**wire_3** is a switch between 2/4-wire operation or 3-wire operation. To choose 2/4-wire operation write false and for 3-wire operation write true. 

----------------------------------------------------
```c
max31865_init(..., bool filter_50Hz, ...);
```
**filter_50Hz** is a switch for the common mode filter. Either 50Hz or 60Hz supression can be choosen. Note that the conversion time differs between 50Hz and 60Hz. The following shows what delay times are needed...

* 50 Hz -> 62.5 ms
* 60 Hz -> 52 ms

----------------------------------------------------

### Measuring Temperature

There are four ways one can get the remperature.

* as raw ADC value with
  ```c
  uint16_t max31865_readADC(const max31865_t* device);
  ```
* as Resistance Value in Ω with
  ```c
  float max31865_readRTD_ohm(const max31865_t* device);
  ```
* as Celsius Value in °C
  ```c
  float max31865_readCelsius(const max31865_t* device);
  ```
* as Kelvin Value in °K
  ```c
  float max31865_readKelvin(const max31865_t* device);
  ```

For reading Celsius °C the Resistance of the RTD is measured and is then calculated with a polynomial approximation of the RTD curve. It is done with the  Horners method to reduce the needed multiplication and addition numbers.

For reading Kelvin °K the measured Value in °C is added with the offset of 273.15 °K.

### Automatic Threshold Fault Detection

Each time when a mesurement is done (raw ADC, Ω, °C or °K) the value is checked if it lies within the boundarys set by the upper and lower Threshold values. If the measured Value is over the upper threshold the **highFaultThreshold_callback** is called. If the value is below the lower threshold the **lowFaultThreshold_callback** is calles. After completion of the callbacks all Faults are cleared.

## Changig the Threshold values

It is possible to change the upper and lower threshold at anny given time by calling the following functions...

```c
void max31865_setHighFaultThreshold(max31865_t* device,
                                    uint16_t    threshold);

void max31865_setLowFaultThreshold(max31865_t*  device,
                                   uint16_t     threshold);

```

## Contributing

Please read [CONTRIBUTING.md]() for details on my code of conduct, and the process for submitting pull requests to us.

## Authors

* **Edwin Koch** - *Initial work* - [polymurph](https://github.com/polymurph)

See also the list of [contributors](https://github.com/polymurph/MAX31865/graphs/contributors) who participated in this project.

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments
* 
