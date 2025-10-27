.. zephyr:code-sample:: blinky 2 threads
   :name: Blinky 2 threads
   :relevant-api: gpio_interface

   Blink 2 LEDs forever using the GPIO API.

Overview
********

The Blinky sample blinks 2 LEDs forever using the :ref:`GPIO API <gpio_api>`.

the idea is to have two threads, each one blinking a different LED. Use systemView to see the two threads running.
When the scheduler is called and why ?
why after the timer come back directly to the thread without passing from scheduler ? and why after the thread it go to the scheduler ?
