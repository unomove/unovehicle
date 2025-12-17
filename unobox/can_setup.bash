sudo busybox devmem 0x0243d008 32 0x400 #PADCTL_G2_SPI1_CS0_0
sudo busybox devmem 0x0243d018 32 0x458 #PADCTL_G2_SPI1_MISO_0
sudo busybox devmem 0x0243d028 32 0x400 #PADCTL_G2_SPI1_SCK_0
sudo busybox devmem 0x0243d040 32 0x400 #PADCTL_G2_SPI1_MOSI_0

sudo ifconfig can0 down
sudo ifconfig can1 down
sudo ip link set can0 up type can bitrate 1000000 dbitrate 2000000 restart-ms 1000 berr-reporting on fd on
sudo ip link set can1 up type can bitrate 1000000 dbitrate 2000000 restart-ms 1000 berr-reporting on fd on
sudo ifconfig can0 txqueuelen 65536
sudo ifconfig can1 txqueuelen 65536