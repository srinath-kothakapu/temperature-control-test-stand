LIB := ../../firmware/lib/control
CFLAGS := -std=c11 -Wall -Wextra -Werror -O1 -I$(LIB) -I../../firmware/include
SRCS := test_control.c $(LIB)/thermocouple.c $(LIB)/controller.c $(LIB)/safety.c $(LIB)/frame.c

test: test_control
	./test_control

test_control: $(SRCS) $(wildcard $(LIB)/*.h)
	$(CC) $(CFLAGS) -o $@ $(SRCS) -lm

clean:
	rm -f test_control

.PHONY: test clean
