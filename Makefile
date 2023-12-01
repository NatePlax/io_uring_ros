all: record sync throttler publisher multirecord multisync

publisher: publisher.cpp
	g++ publisher.cpp -o publisher -I/opt/ros/noetic/include -L/opt/ros/noetic/lib/ -lroscpp -lrostime -lrosconsole -lroscpp_serialization

record: record.cpp
	g++ record.cpp -o record -I/opt/ros/noetic/include -L/opt/ros/noetic/lib/ -lroscpp -lrostime -lrosconsole -lroscpp_serialization -luring

multirecord: multi_record.cpp
	g++ multi_record.cpp -o multirecord -I/opt/ros/noetic/include -L/opt/ros/noetic/lib/ -lroscpp -lrostime -lrosconsole -lroscpp_serialization -luring

sync: sync_record.cpp
	g++ sync_record.cpp -o sync -I/opt/ros/noetic/include -L/opt/ros/noetic/lib/ -lroscpp -lrostime -lrosconsole -lroscpp_serialization -luring

multisync: multi_sync.cpp
	g++ multi_sync.cpp -o multisync -I/opt/ros/noetic/include -L/opt/ros/noetic/lib/ -lroscpp -lrostime -lrosconsole -lroscpp_serialization -luring

throttler: throttler.cpp
	g++ throttler.cpp -lpthread -o throttler

clean:
	rm record sync throttler output* publisher multirecord multisync
