# Builds and runs the matcher's verification suite on this PC.
#
# flow_match.c has no ESP-IDF dependency precisely so this is possible: the
# algorithm that will run on the camera is tested here, against shifts whose
# true value we chose, with no hardware involved.

CFLAGS := -O2 -Wall -Wextra -Wconversion -std=c11
LDLIBS := -lm

.PHONY: all test sweep clean
all: test_flow_match bias_sweep

test_flow_match: test_flow_match.c ../main/flow_match.c ../main/flow_match.h
	$(CC) $(CFLAGS) -o $@ test_flow_match.c ../main/flow_match.c $(LDLIBS)

bias_sweep: bias_sweep.c ../main/flow_match.c ../main/flow_match.h
	$(CC) $(CFLAGS) -o $@ bias_sweep.c ../main/flow_match.c $(LDLIBS)

test: test_flow_match
	./test_flow_match

sweep: bias_sweep
	./bias_sweep

clean:
	rm -f test_flow_match bias_sweep
