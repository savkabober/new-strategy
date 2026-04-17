cmake_minimum_required(VERSION 3.10)
project(OptimizationApp)

# Стандарт C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Оптимизация для Release сборки
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2")

# 1. Ищем NLopt
find_package(NLopt REQUIRED)

# 2. Ищем SFML (добавляем эту часть)
# Указываем компоненты, которые нам нужны
find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)

# Описываем исполняемый файл
add_executable(app main.cpp)

# 3. Подключаем библиотеки
target_link_libraries(app 
    PRIVATE 
        sfml-graphics 
        sfml-window 
        sfml-system
        NLopt::nlopt
)

target_include_directories(app PRIVATE ${SFML_INCLUDE_DIR})