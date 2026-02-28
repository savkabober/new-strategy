cmake_minimum_required(VERSION 3.10)
project(OptimizationApp)

# Устанавливаем стандарт C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Оптимизация (аналог -O3)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

# Ищем NLopt
find_package(NLopt REQUIRED)

# Описываем исполняемый файл
add_executable(app main.cpp)

# Подключаем библиотеки к нашему приложению
target_link_libraries(app 
    PRIVATE 
        sfml-graphics 
        sfml-window 
        sfml-system
        NLopt::nlopt
)

# Если SFML установлен нестандартно, это поможет найти его заголовки
target_include_directories(app PRIVATE ${SFML_INCLUDE_DIR})