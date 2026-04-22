CXX = g++
CXXFLAGS = -Wall
LIBS = -lsfml-graphics -lsfml-window -lsfml-system -lnlopt
TARGET = app
SRC = main.cpp

$(TARGET): $(SRC)
	$(CXX) $(SRC) -o $(TARGET) $(LIBS) $(CXXFLAGS) #&& .\$(TARGET)

clean:
	rm -rf $(TARGET)
