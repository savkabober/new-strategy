CXX = g++
CXXFLAGS = -Wall -O3
LIBS = -lsfml-graphics -lsfml-window -lsfml-system -lnlopt
TARGET = app.exe
SRC = main.cpp

$(TARGET): $(SRC)
	$(CXX) $(SRC) -o $(TARGET) $(CXXFLAGS) $(LIBS) && .\$(TARGET)



clean:
	del $(TARGET)
