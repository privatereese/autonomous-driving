TARGET = ecu

SRC_CC = main.cc Parking.cc ecu.cc
SRC_CC += $(addprefix $(REP_DIR)/../../tools/QEMU-SA-VM/src/app/savm/, subscriber.cc publisher.cc)


LIBS  = base libmosquitto stdcxx libc lwip pthread
LIBS  += syscall-foc

INC_DIR += $(call select_from_repositories,include/lwip)
INC_DIR += $(REP_DIR)/../../tools/QEMU-SA-VM/src/app/savm
