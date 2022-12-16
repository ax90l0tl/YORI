py_pubsub is just the publisher subscriber code from the ros2 tutorials but there are some modifications to the subscriber function

The subscriber uses pyserial to send a message to the Teensy Serial Interface. Every time the subscriber receives a message from the publisher, it sends a serial message.
The Teensy must be connected to the computer with the serial port open for this to work.
Also the device port might differ between computers.
