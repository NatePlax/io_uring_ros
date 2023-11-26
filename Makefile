all: record sync throttler

record: record.cpp
	g++ record.cpp -o record -I/opt/ros/noetic/include -L/opt/ros/noetic/lib/ -lroscpp -lrostime -lrosconsole -lroscpp_serialization -luring

sync: sync_record.cpp
	g++ sync_record.cpp -o sync -I/opt/ros/noetic/include -L/opt/ros/noetic/lib/ -lroscpp -lrostime -lrosconsole -lroscpp_serialization -luring

throttler: throttler.cpp
	g++ throttler.cpp -lpthread -o throttler

clean:
	rm record sync throttler
