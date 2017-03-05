
default: all

all:  libmb.a libmb.so

OBJ_FILES = mb_ascii.o mb_rtu.o mb_tcp.o mb_master.o mb_slave.o mb_slave_and_master.o sin_util.o

libmb.a:  $(OBJ_FILES)
	ar cr libmb.a $(OBJ_FILES)

libmb.so: $(OBJ_FILES)
	gcc -shared -fPIC -o libmb.so $(OBJ_FILES)

clean:
	-rm -rf *.o libmb.a libmb.so



# use gcc
CC = gcc

#get warnings, debugging information and optimization
CFLAGS  = -Wall -Wpointer-arith -Wstrict-prototypes -Wwrite-strings
# CFLAGS += -Werror
CFLAGS += -ggdb -O3 -funroll-loops
# Note: if the optimizer crashes, we'll leave out the -O3 for those files

# Required for compilation with beremiz, and to create shared object library
CFLAGS += -fPIC



#how to make things from other directories if they are missing
../% /%:
	$(MAKE) -C $(@D) $(@F)

Makefile.depend depend:
#	gcc -MM -MG -I$(LLIB) *.c \
	gcc -MM -MG *.c \
		| perl -pe 's/:/ Makefile.depend:/' \
		> Makefile.depend

include Makefile.depend

