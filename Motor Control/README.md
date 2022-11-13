# roboclaw_driver
ROS driver for the Roboclaw motor controller.

## Parameters

See the included launchfile for an example.

* ~dev - name of the serial device
* ~baud - baud rate, in bps (default: 38400, not important for USB)
* ~address - address of the Roboclaw board (default: 128)
* ~qp_per_meter - encoder resolution, quadrature pulses per meter
* ~base_width - distance between the right and left wheel, in meters
* ~p, ~i, ~d - PID gains
* ~max_qpps - quadrature pulses per second when the motor is at 100% power
* ~max_speed - software speed limit to enforce, in m/s (default: 2.0)
