# Builds and runs the detector's verification suite on this PC.
#
# blob_detect.c has no ESP-IDF dependency precisely so this is possible: the
# code that will run on the camera is tested here against synthetic frames
# whose answer we chose. AddressSanitizer is on because test 9 drives the label
# arrays to their worst case, and an overrun there must fail loudly.

CFLAGS := -O1 -g -Wall -Wextra -Wconversion -std=c11 -fsanitize=address,undefined
LDLIBS := -lm

.PHONY: all test lib clean
all: test_blob_detect lib

test_blob_detect: test_blob_detect.c ../main/blob_detect.c ../main/blob_detect.h
	$(CC) $(CFLAGS) -o $@ test_blob_detect.c ../main/blob_detect.c $(LDLIBS)

test: test_blob_detect
	./test_blob_detect

# The same detector as a shared library, for the PC viewer (viewer/tcview/
# detector.py loads it with ctypes). No sanitizers: they cannot be loaded
# into a Python process. The viewer rebuilds it when the C source is newer.
lib: libblob_detect.so

libblob_detect.so: ../main/blob_detect.c ../main/blob_detect.h
	$(CC) -O2 -Wall -Wextra -std=c11 -fPIC -shared -o $@ ../main/blob_detect.c $(LDLIBS)

clean:
	rm -f test_blob_detect libblob_detect.so
