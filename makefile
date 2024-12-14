
CXX = g++
CXXFLAGS = -Wall -Wextra -Iinclude


TARGET = nav

SRC = main.cpp


all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) -o $@ $^

clean:
	rm -f $(TARGET)
