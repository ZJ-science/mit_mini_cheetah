#ifndef _rt_serial
#define _rt_serial

// void set_blocking (int fd, int should_block, int port);
int set_interface_attribs_custom_baud(int fd, int speed, int parity, int port);
void init_serial_for_sbus(int fd, int baud);

#endif
