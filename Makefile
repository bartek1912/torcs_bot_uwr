CC            =  g++
CPPFLAGS      = -Wall -g -pedantic -std=c++11 -Iinterfaces 

# Uncomment the following line for a verbose client
#CPPFLAGS      = -Wall -g -D __UDP_CLIENT_VERBOSE__

#Put here the name of your driver class
DRIVER_CLASS = SimpleDriver
#Put here the filename of your driver class header 
DRIVER_INCLUDE = '"$(DRIVER_CLASS).h"' 
DRIVER_OBJ = $(DRIVER_CLASS).o
FORWARD_MODEL_LOCATIONS = forward_model/matrix.o forward_model/car.o forward_model/steer.o forward_model/brake.o forward_model/wheel.o forward_model/quaternion.o forward_model/ForwardModel.o forward_model/engine.o
FORWARD_MODEL_LIST = matrix.o car.o steer.o brake.o wheel.o quaternion.o ForwardModel.o engine.o

EXTFLAGS = -D __DRIVER_CLASS__=$(DRIVER_CLASS) -D __DRIVER_INCLUDE__=$(DRIVER_INCLUDE)

OBJECTS = WrapperBaseDriver.o track_model.o SimpleParser.o CarState.o CarControl.o TrackModel.o $(FORWARD_MODEL_LOCATIONS) $(DRIVER_OBJ)
OBJECTS_LOCATIONS = WrapperBaseDriver.o track_model.o SimpleParser.o CarState.o CarControl.o TrackModel.o  $(FORWARD_MODEL_LIST) $(DRIVER_OBJ)

all: $(OBJECTS) client


.SUFFIXES : .o .cpp .c

.cpp.o :
	@$(CC) $(CPPFLAGS) $(EXTFLAGS) -c $<

.c.o :
	@$(CC) $(CPPFLAGS) $(EXTFLAGS) -c $<


client: client.cpp $(OBJECTS)
	$(CC) $(CPPFLAGS) $(EXTFLAGS) -o client client.cpp $(OBJECTS_LOCATIONS)

clean:
	rm -f *.o client  
