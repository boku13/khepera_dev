#
# libkhepera tests Makefile
#
#
# libkhepera tests Makefile
#
.PHONY: tests clean


SRCS	= $(wildcard *.c)
OBJS	= $(patsubst %.c,%.o,${SRCS})
INCS	=-I../${BUILD}/include
LIBS	=-L../${BUILD}/lib -lkhepera -lm


ifeq ($(DEBUG),1)
CFLAGS 	= -g -fPIC
else
CFLAGS 	= -O3 -fPIC
endif



#Test Programs
TARGETS	= kb_config_test \
	  koreio_test koreio_auto kmot_monitor gpio_test \
	  kmot_ipserver kmotLE_monitor koreioLE_test \
	  koreioLE_auto kgripper_test \
	  klrf_test klrf_small_ex kgazer_test kgazer_small_ex \
	  kh4_example khepera4_test gpio_test_all kh4server kh4_lrf_batpower back circ\
	  kh4_4 kh4_c kh4_c2 kh4_circle linear_test turn_180 spin turn_test test1
	  
#Test Programs which require pthread
PTHREAD_TARGETS = kmot_test kmotLE_test

.PHONY: test clean depend 

all: ${TARGETS} ${PTHREAD_TARGETS}

test: ${TARGETS} ${PTHREAD_TARGETS}

$(TARGETS): % : %.o	
ifeq ($(DEBUG),1)
	@echo "Building $@ (DEBUG mode)"
else
	@echo "Building $@"
endif
	@$(CC) -o $@ $? $(LIBS) $(CFLAGS) 
	

$(PTHREAD_TARGETS): % : %.o
ifeq ($(DEBUG),1)
	@echo "Building $@ (DEBUG mode)"
else
	@echo "Building $@"
endif
	@$(CC) -o $@ $? -pthread $(LIBS) $(CFLAGS) 
	

clean: 
	@echo "Cleaning"
	@rm -f *.o .depend ${TARGETS} ${PTHREAD_TARGETS} *~

depend:	
	@echo "Building dependencies"
	@rm -f .depend
	@touch .depend
	@makedepend ${SYS_INCLUDES} -I .. -Y -f .depend ${SRCS}

%.o:	%.c
	@echo "Compiling $@"
	@$(CC) $(INCS) -c $(CFLAGS) $< -o $@

ifeq (.depend,$(wildcard .depend))
include .depend 
endif
