.. zephyr:board:: arduino_nano_esp32

Overview
********

The `Arduino Nano ESP32`_ is a compact development board leveraging the
powerful ESP32-S3 microcontroller. It offers built-in Wi-Fi and Bluetooth
connectivity, making it ideal for IoT projects. Perfect for embedding in
standalone projects, it provides versatility for beginners and experienced
makers alike.

Hardware
********

- u-blox NORA-W106-10B module with ESP32-S3 CPU, 8MB PSRAM, antenna
- 16MB SPI flash memory
- One RGB LED and one user LED
- One reset button
- Native USB-C port for power and programming

Asymmetric Multiprocessing (AMP)
********************************

The ESP32-S3 allows 2 different applications to be executed in ESP32-S3 SoC. Due to its dual-core
architecture, each core can be enabled to execute customized tasks in stand-alone mode
and/or exchanging data over OpenAMP framework. See :zephyr:code-sample-category:`ipc` folder as code reference.

For more information, check the datasheet at `ESP32-S3 Datasheet`_ or the technical reference
manual at `ESP32-S3 Technical Reference Manual`_.

Supported Features
==================

.. zephyr:board-supported-hw::

Prerequisites
-------------

Espressif HAL requires WiFi and Bluetooth binary blobs in order work. Run the command
below to retrieve those files.

.. code-block:: console

   west blobs fetch hal_espressif

.. note::

   It is recommended running the command above after :file:`west update`.

Building & Flashing
*******************

.. zephyr:board-supported-runners::

Manual build
============

Build and flash applications as usual (see :ref:`build_an_application` and
:ref:`application_run` for more details).

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: arduino_nano_esp32/esp32s3/procpu
   :goals: build

The usual ``flash`` target will work with the ``arduino_nano_esp32`` board
configuration. Here is an example for the :zephyr:code-sample:`hello_world`
application.

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: arduino_nano_esp32/esp32s3/procpu
   :goals: flash

Open the serial monitor using the following command:

.. code-block:: shell

   west espressif monitor

After the board has automatically reset and booted, you should see the following
message in the monitor:

.. code-block:: console

   ***** Booting Zephyr OS vx.x.x-xxx-gxxxxxxxxxxxx *****
   Hello World! arduino_nano_esp32

Debugging
*********

ESP32-S3 support on OpenOCD is available at `OpenOCD ESP32`_.

ESP32-S3 has a built-in JTAG circuitry and can be debugged without any additional chip. Only an USB cable connected to the D+/D- pins is necessary.

Further documentation can be obtained from the SoC vendor in `JTAG debugging for ESP32-S3`_.

Here is an example for building the :zephyr:code-sample:`hello_world` application.

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: arduino_nano_esp32/esp32s3/procpu
   :goals: build flash

You can debug an application in the usual way. Here is an example for the :zephyr:code-sample:`hello_world` application.

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: arduino_nano_esp32/esp32s3/procpu
   :goals: debug

References
**********

.. target-notes::

.. _`Arduino Nano ESP32`: https://docs.arduino.cc/hardware/nano-esp32/
.. _`ESP32-S3 Datasheet`: https://www.espressif.com/sites/default/files/documentation/esp32-s3-wroom-1_wroom-1u_datasheet_en.pdf
.. _`ESP32-S3 Technical Reference Manual`: https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf
.. _`JTAG debugging for ESP32-S3`: https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-guides/jtag-debugging/
.. _`OpenOCD ESP32`: https://github.com/espressif/openocd-esp32/releases
