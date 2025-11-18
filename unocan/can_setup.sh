sudo modprobe can-raw
sudo modprobe can-dev
sudo modprobe mttcan
sudo rmmod mcp251xfd.ko
sudo insmod /lib/modules/$(uname -r)/kernel/drivers/net/can/spi/mcp251xfd.ko

sudo ifconfig can0 down
sudo ifconfig can1 down

sudo ip link set can0 up type can bitrate 500000
sudo ip link set can1 up type can bitrate 500000

sudo ip link set up can0
sudo ip link set up can1

sudo ifconfig can0 txqueuelen 65536
sudo ifconfig can1 txqueuelen 65536
