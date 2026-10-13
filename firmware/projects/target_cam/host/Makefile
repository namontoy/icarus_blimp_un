# Builds and runs the detector's verification suite on this PC.
#
# blob_detect.c has no ESP-IDF dependency precisely so this is possible: the
# code that will run on the camera is tested here against synthetic frames
# whose answer we chose. AddressSanitizer is on because test 9 drives the label
# arrays to their worst case, and an overrun there must fail loudly.

CFLAGS := -O1 -g -Wall -Wextra -Wconversion -std=c11 -fsanitize=address,undefined
LDLIBS := -lm

.PHONY: all test clean
all: test_blob_detect

test_blob_detect: test_blob_detect.c ../main/blob_detect.c ../main/blob_detect.h
	$(CC) $(CFLAGS) -o $@ test_blob_detect.c ../main/blob_detect.c $(LDLIBS)

test: test_blob_detect
	./test_blob_detect

clean:
	rm -f test_blob_detect
