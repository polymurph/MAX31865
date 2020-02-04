# MAX31865

This is a driver for the MAX31865 RTD-to-Digital converter (RTD: resistance temperature detectors).

## Getting Started

These instructions whil help you implement the drive in your project step by step.

### Prerequisites

Download the max31865.c/.h files into a desired folder inside your project.

### Creating an Oject

The driver has an object-oriented approach which enbales a certain abstraction which helps to minimize complexety and improve readability.

Firts we have to include the driver as following...

```c
#include "<your path>/max31865.h"
```

Then we can create an instance...

```c
max31865_t TempSensor;
```

This instance has not yet been initialized. To need the following callbacks 


## Contributing

Please read [CONTRIBUTING.md]() for details on my code of conduct, and the process for submitting pull requests to us.

## Versioning

We use [SemVer](http://semver.org/) for versioning. For the versions available, see the [tags on this repository](https://github.com/your/project/tags). 

## Authors

* **Edwin Koch** - *Initial work* - [polymurph](https://github.com/polymurph)

See also the list of [contributors](https://github.com/your/project/contributors) who participated in this project.

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

* Hat tip to anyone whose code was used
* Inspiration
* etc
