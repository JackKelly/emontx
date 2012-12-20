compile = g++ -Wall -g -c -O0 -D TESTING -I/home/jack/workspace/avr/emontx $<
link = g++ -Wall -g -O0 -D TESTING 

#-fno-elide-constructors
# -I/usr/lib/avr/include/ -D UBRRH -I/usr/lib/avr/include/avr/pgmspace.h -I/usr/share/arduino/hardware/arduino/cores/arduino/ -I/usr/share/arduino/hardware/arduino/variants/standard/

all: CircularBuffer_test

CircularBuffer_test: CircularBuffer_test.o
	$(link) $^ -lboost_unit_test_framework -o $@ && ./$@

CircularBuffer_test.o: CircularBuffer_test.cpp ../CircularBuffer.h
	$(compile)

	
clean:
	rm -rf *.o CircularBuffer_test