# MAX31865

This is a driver for the [MAX31865](https://www.maximintegrated.com/en/products/interface/sensor-interface/MAX31865.html) RTD-to-Digital converter (RTD: resistance temperature detectors).

## Getting Started

These instructions whil help you implement the drive in your project step by step.

The driver has an object-oriented approach which enbales a certain abstraction which helps to minimize complexety and improve readability.

### Prerequisites

[Download](https://github.com/polymurph/MAX31865/archive/master.zip) the max31865.c/.h files into a desired folder inside your project.

### Creating an Oject

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
max31865_init(TempSensor,...);
```
The init function will update all the struct members of TempSensor for you. It will also write the desired configurations and the upper and lower temperature fault thresholds directly to the device (MAX31865).



## Contributing

Please read [CONTRIBUTING.md]() for details on my code of conduct, and the process for submitting pull requests to us.

## Authors

* **Edwin Koch** - *Initial work* - [polymurph](https://github.com/polymurph)

See also the list of [contributors](https://github.com/polymurph/MAX31865/graphs/contributors) who participated in this project.

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

* Hat tip to anyone whose code was used
* Inspiration
* etc
