.. zephyr:code-sample:: mutex
   :name: Blinky
   :relevant-api: gpio_interface

   share a resource (LED) between a thread  with priority 5 and another thread with priority 14.

Overview
********

this example application demonstrates the use of mutexes to share a resource (LED) between a
thread with priority 5 and another thread with priority 14.

this idea arises from the fact when a resource is shared between an application thread and the shell thread,
the shell thread has a priority of 14, and the application thread has a priority of 5.
so we can say than the applications thread has a higher priority than the shell thread. so it will never be preempted by the shell thread.
However, what happens when is running the shell thread and is preempted by the application thread, and the application thread put the register of the led to 1 (led on),
and then the scheduler come back to the shell thread, and the shell thread put the register of the led to 0 (led off),

so without the mutex is always of the LED
