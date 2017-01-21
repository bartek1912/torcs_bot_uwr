CC            =  g++
CPPFLAGS      = -Wall -g -std=c++11 

OBJECTS = matrix.o car.o steer.o brake.o wheel.o quaternion.o

.SUFFIXES : .o .cpp .c

.cpp.o :
	$(CC) $(CPPFLAGS) $(EXTFLAGS) -c $<

.c.o :
	$(CC) $(CPPFLAGS) $(EXTFLAGS) -c $<


forward: ForwardModel.cpp $(OBJECTS)
		$(CC) $(CPPFLAGS) -c -o ForwardModel.o ForwardModel.cpp $(OBJECTS)

clean:
	rm -f *.o  
