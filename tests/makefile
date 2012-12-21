# DIRECTORIES
nanode_rf_utils_dir = /home/jack/workspace/avr/nanode_rf_utils
emon_tx_dir = /home/jack/workspace/avr/emontx

# COMPILATION AND LINKING VARIABLES
CXX = g++
CXXFLAGS := -Wall -MMD -g -O0 -D TESTING -I$(emon_tx_dir) -I$(nanode_rf_utils_dir)
LINK = ${CXX} $^ -lboost_unit_test_framework -o $@ && ./$@

# TARGETS
EXECS = CircularBuffer_test

# RULE FOR all
all: $(EXECS)

# DEPENDENCIES FOR LINKING STEP
CircularBuffer_test: CircularBuffer_test.o


# LINKING STEP:
$(EXECS):
	${CXX} $^ -lboost_unit_test_framework -o $@ && ./$@

# INCLUDE COMPILATION DEPENDENCIES
-include *.d
-include ../*.d

# Clean
clean:
	rm -rf *.o *_test *.d ../*.o ../*.d