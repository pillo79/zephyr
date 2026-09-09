.. _external_module_arduino_core_api:

Arduino Core API
################

Introduction
************

The ArduinoCore-Zephyr module started as a `Google Summer of Code 2022 project`_
to provide Arduino-style APIs for Zephyr RTOS applications. This module acts as an abstraction
layer, allowing developers familiar with Arduino programming to leverage Zephyr's capabilities
without having to learn entirely new APIs and libraries.

Understanding the Components
============================

This module consists of two key components that work together:

**1. ArduinoCore-API (Common Arduino API Definition)**

The `ArduinoCore-API <https://github.com/arduino/ArduinoCore-API>`_ is Arduino's official
hardware abstraction layer that defines the common Arduino API. It contains the abstract API
definitions **and** implementations for hardware-independent functionality.

Key characteristics:

* Contains both API definitions (headers) and hardware-agnostic implementations
* Provides classes like ``String``, ``Print``, ``Stream``, ``IPAddress`` with full implementations
* Defines interfaces for hardware-specific classes (e.g., ``HardwareSerial``, ``HardwareSPI``)
* Shared across all modern Arduino platforms for consistency
* See the `ArduinoCore-API README <https://github.com/arduino/ArduinoCore-API#arduinocore-api>`_ for implementation details
* It is Licensed as GNU LESSER GENERAL PUBLIC LICENSE, version 2.1 or later (LGPL-2.1+)

**2. ArduinoCore-Zephyr (Zephyr-Specific Implementation)**

The `ArduinoCore-Zephyr <https://github.com/zephyrproject-rtos/ArduinoCore-zephyr>`_ module
provides the **Zephyr-specific implementation** of the Arduino API. This is where hardware-dependent
Arduino functions are implemented using Zephyr's native APIs and drivers.

Key characteristics:

* Contains the ``cores/arduino`` directory with Zephyr implementations (``zephyrCommon.cpp``, ``zephyrSerial.cpp``, etc.)
* Provides board-specific variants with pin mappings and Device Tree overlays
* Includes Zephyr build system integration (CMake, Kconfig, west.yml)
* Links to ArduinoCore-API to inherit the common implementations
* See the `project documentation <https://github.com/zephyrproject-rtos/ArduinoCore-zephyr/tree/main/documentation>`_ for implementation details
* It is Licensed under Apache-2.0

Features Provided
=================

Together, these components provide:

* Standard Arduino API functions like ``pinMode()``, ``digitalWrite()``, ``analogRead()``, etc.
* Support for Arduino-style ``setup()`` and ``loop()`` functions
* Pin mapping between Arduino pin numbers and Zephyr's GPIO definitions
* Support for common Arduino communication protocols (Serial, I2C, SPI)
* Compatibility with existing Arduino libraries
* Board variant support for various hardware platforms already present in Zephyr

By bringing Arduino-style programming to Zephyr, this module provides a gentler learning curve
for those transitioning from Arduino to Zephyr while still benefiting from Zephyr's advanced
features, scalability, and broad hardware support.

Usage with Zephyr
*****************

Adding the Arduino Core API to a Zephyr Project
===============================================

#. To pull in the Arduino Core for Zephyr as a Zephyr module, either add it as
   a West project in the west.yml file or pull it in by adding a submanifest
   (e.g. ``zephyr/submanifests/arduinocore.yaml``) file with the following content:

   .. code-block:: yaml

      # Arduino API repository
      - name: ArduinoCore-zephyr
        path: modules/lib/arduinocore-zephyr
        revision: main
        url: https://github.com/zephyrproject-rtos/ArduinoCore-zephyr

#. Run the following command to update your project:

   .. code-block:: bash

      west update

#. Complete the core setup by downloading the Arduino API and copying it into the
   ``cores/arduino/api`` folder of the module:

   .. code-block:: bash

      west blobs fetch

   The ``cores`` folder is located inside ``<zephyr-project-path>/modules/lib/arduinocore-zephyr/cores``.

Using Arduino Core API in Your Application
==========================================

#. In your application's ``prj.conf`` file, enable the Arduino API configs similar to how it's being
   done in the `blinky_arduino sample`_.

#. Create your application using Arduino-style code:

   .. code-block:: cpp

      #include <Arduino.h>

      void setup() {
        pinMode(LED_BUILTIN, OUTPUT);
      }

      void loop() {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(1000);
        digitalWrite(LED_BUILTIN, LOW);
        delay(1000);
      }

#. Build your application with the target board:

   .. code-block:: bash

      west build -b <board_name> path/to/your/app

Adding Custom Board Support
===========================

Supported boards reside in the ``variants/`` directory in the arduinocore-zephyr.
To add support for a custom board:

#. Create a new folder in the ``variants/`` directory with your board's name
#. Add an overlay file and a pinmap header file that match the board name
#. Add your new header file to an ``#ifdef`` statement in the ``variant.h`` file

For detailed instructions on adding board variants, refer to the `board variants documentation`_.

Using External Arduino Libraries
================================

To use external Arduino libraries with your Zephyr project:

#. Add your library's source files (e.g., ``MyLibrary.h`` and ``MyLibrary.cpp``) to your project's ``src`` folder
#. Update your application's ``CMakeLists.txt`` to include these files:

   .. code-block:: cmake

      target_sources(app PRIVATE src/MyLibrary.cpp)

#. Include the library in your source code:

   .. code-block:: cpp

      #include "MyLibrary.h"

For more details on using external libraries, see the `Arduino libraries documentation`_.

Official Arduino downstream fork
********************************

Arduino maintains a `downstream fork of the ArduinoCore-Zephyr repository
<https://github.com/arduino/ArduinoCore-zephyr>`_ that provides full integration with the Arduino
ecosystem of tools, generating Arduino IDE-compatible core packages. This fork is maintained by
Arduino and is intended for Arduino users who have no previous knowledge of Zephyr and just want to
use their board within the Arduino ecosystem.

To achieve this, the Arduino downstream fork provides an universal 'loader' Zephyr application that
is pre-compiled for each supported board and distributed as a binary. When flashed to an actual
board, this allows the Arduino IDE to upload user sketches which are then run by Zephyr via LLEXT.
With this split, the loader is a static C application, while all C++ code in the core and APIs is
linked with the user sketch by the IDE.

In this way, basic Arduino users can benefit from Zephyr without having to learn about it at all,
while users who dive into the workings of the core can start using Zephyr's native APIs and features
and ultimately transition to building Zephyr applications directly.

References
**********

#. `ArduinoCore-Zephyr GitHub Repository`_
#. `ArduinoCore-API Repository`_
#. `Golioth Article: Zephyr + Arduino: a Google Summer of Code story`_

.. target-notes::

.. _Arduino Core API: https://github.com/zephyrproject-rtos/ArduinoCore-zephyr
.. _board variants documentation: https://github.com/zephyrproject-rtos/ArduinoCore-zephyr/blob/main/documentation/variants.md
.. _Arduino libraries documentation: https://github.com/zephyrproject-rtos/ArduinoCore-zephyr/blob/main/documentation/arduino_libs.md
.. _ArduinoCore-Zephyr GitHub Repository: https://github.com/zephyrproject-rtos/ArduinoCore-zephyr
.. _ArduinoCore-API Repository: https://github.com/arduino/ArduinoCore-API
.. _Google Summer of Code 2022 project: https://dhruvag2000.github.io/Blog-GSoC22/
.. _Golioth Article\: Zephyr + Arduino\: a Google Summer of Code story: https://blog.golioth.io/zephyr-arduino-a-google-summer-of-code-story/
.. _blinky_arduino sample: https://github.com/zephyrproject-rtos/ArduinoCore-zephyr/blob/next/samples/blinky_arduino/prj.conf
