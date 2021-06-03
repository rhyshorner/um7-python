# um7-python
a module to communicate to um7 with serial bus

# red-shift lab
- https://redshiftlabs.com.au/product/um7-lt-orientation-sensor/
- https://redshiftlabs.com.au/support-services/serial-interface-software/
- use serial interface software to stop all automated broadcast messages so module can use master/slave style communication.

This is a class which uses module um7.py to utilize in miniray project

# if error comes up;
AttributeError: module 'serial' has no attribute 'Serial'" exist
then run;
$ pip3 uninstall serial

requirements
-   pyserial

rpi config required:
-   enable serial
-   disable serial console

had to fix up at one stage;
-   ~$ sudo nano /boot/config.txt
added line;
-   ~$ core_freq=250
this made random rpi serial frequency changing stop