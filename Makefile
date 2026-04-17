# Имя выходного файла
TARGET = app

# Компилятор и флаги
CXX = g++
CXXFLAGS = -Wall -Wextra -pedantic -O3 -std=c++17
LDFLAGS =
LDLIBS =

SFML_AVAILABLE := $(shell if pkg-config --exists sfml-graphics; then echo 1; elif [ -d /usr/include/SFML ]; then echo 1; else echo 0; fi)

ifeq ($(SFML_AVAILABLE),1)
SFML_CFLAGS := $(shell pkg-config --cflags sfml-graphics 2>/dev/null)
SFML_LIBS := $(shell pkg-config --libs sfml-graphics 2>/dev/null)
ifeq ($(strip $(SFML_LIBS)),)
SFML_LIBS := -lsfml-graphics -lsfml-window -lsfml-system
endif
CXXFLAGS += $(SFML_CFLAGS)
LDLIBS += $(SFML_LIBS)
endif

# Исходники
SRC = main.cpp

# Правило по умолчанию
all: $(TARGET)

# Правило сборки
$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $(SRC) -o $(TARGET) $(LDFLAGS) $(LDLIBS)
init:
	@echo "Install SFML for live visualization: sudo apt install libsfml-dev"
# Очистка
clean:
	rm -f $(TARGET)
