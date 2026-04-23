CXX = g++
CXXFLAGS = -Wall
LIBS = -lsfml-graphics -lsfml-window -lsfml-system -lnlopt
TARGET = app.exe
SRC = main.cpp

$(TARGET): $(SRC)
	$(CXX) $(SRC) -o $(TARGET) $(CXXFLAGS) $(LIBS)

run: $(TARGET)
	.\$(TARGET)


clean:
	del $(TARGET)
