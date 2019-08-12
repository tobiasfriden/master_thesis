MASON=/mason/mason
VARIANT = variant 1.1.4
GEOMETRY = geometry 1.0.0
RAPIDJSON = rapidjson 1.1.0
GEOJSON = geojson 0.4.3
BOOST = boost 1.67.0

DEPS = `$(MASON) cflags $(VARIANT)` `$(MASON) cflags $(GEOMETRY)` `$(MASON) cflags $(RAPIDJSON)` `$(MASON) cflags $(GEOJSON)` `$(MASON) cflags $(BOOST)`
DEPS += -I./third_party/include
DEPS += -I.

FILES = astar.cpp

LIBS = ./third_party/lib/libs2.a -lcrypto -lssl -pthread

FLAGS = -Wno-attributes -Wno-deprecated-declarations

default:
	g++ -std=c++14 $(DEPS) main.cpp $(FILES) $(LIBS) $(FLAGS)

test:
	g++ -std=c++14 $(DEPS) test.cpp $(FILES) $(LIBS) $(FLAGS)

debug:
	g++ -g -std=c++14 $(DEPS) main.cpp $(FILES) $(LIBS) $(FLAGS)