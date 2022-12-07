sudo touch /etc/udev/rules.d/77-spacemouse.rules

echo "KERNEL==\"hidraw*\", ATTRS{idVendor}==\"256f\", ATTRS{idProduct}==\"c62e\", MODE=\"0666\", GROUP=\"plugdev\"" > /etc/udev/rules.d/77-spacemouse.rules

echo "SUBSYSTEM==\"usb\", ATTRS{idVendor}==\"256f\", ATTRS{idProduct}==\"c62e\", MODE=\"0666\", GROUP=\"plugdev\"" >> /etc/udev/rules.d/77-spacemouse.rules

sudo udevadm control --reload-rules
