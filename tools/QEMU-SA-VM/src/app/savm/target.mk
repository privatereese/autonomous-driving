proto_cc := $(wildcard $(REP_DIR)/../protobuf/build/*.pb.cc)
proto_h := $(REP_DIR)/../protobuf/build
TARGET = savm
SRC_CC = main.cc $(proto_cc) savm.cc
SRC_CC += $(addprefix $(REP_DIR)/../../tools/QEMU-SA-VM/src/app/savm/, subscriber.cc publisher.cc)
LIBS   = base libmosquitto stdcxx libc lwip pthread libprotobuf
LIBS  += syscall-foc

INC_DIR += $(call select_from_repositories,include/lwip)
INC_DIR += $(proto_h) $(REP_DIR)
