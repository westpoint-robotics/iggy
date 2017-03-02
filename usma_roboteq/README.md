udev rule is:
SUBSYSTEM=="tty", ATTRS{idProduct}=="5740", ATTRS{idVendor}=="20d2", SYMLINK+="roboteq"

TODO: Use C API to send setup commands to the controller.
