# Имя выходного файла
TARGET = app

# Компилятор и флаги
CXX = g++
CXXFLAGS = -Wall -o3 -std=c++17

# SFML библиотеки
LIBS = -lsfml-graphics -lsfml-window -lsfml-system

# Исходники
SRC = main.cpp

# Правило по умолчанию
all: $(TARGET)

# Правило сборки
$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $(SRC) -o $(TARGET) $(LIBS)
init:
	sudo apt install libsfml-dev
# Очистка
clean:
	rm -f $(TARGET)
