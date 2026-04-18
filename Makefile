CXX = g++
CXXFLAGS = -Wall
LIBS = -lsfml-graphics -lsfml-window -lsfml-system
TARGET = app.exe
SRC = main.cpp

$(TARGET): $(SRC)
	$(CXX) $(SRC) -o $(TARGET) $(LIBS) $(CXXFLAGS) && .\app.exe

clean:
	del $(TARGET)
