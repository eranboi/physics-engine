cmake_minimum_required(VERSION 3.14)

# Proje ismin ve versiyonun
project(physics-engine VERSION 1.0 LANGUAGES CXX)

# C++ standardını belirle (C++17 modern oyun dev için idealdir)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- SFML'i Otomatik İndir ve Kur (FetchContent) ---
include(FetchContent)

FetchContent_Declare(
    sfml
    GIT_REPOSITORY https://github.com/SFML/SFML.git
    GIT_TAG        3.0.2
)

# SFML'i indir ve projeye dahil et
# (Bu işlem ilk build'de biraz uzun sürer çünkü SFML'i derler)
FetchContent_MakeAvailable(sfml)

# --- Kaynak Dosyaları ---
# src klasöründeki tüm .cpp dosyalarını bul
file(GLOB_RECURSE SOURCES "src/*.cpp")

# --- Executable Oluştur ---
add_executable(${PROJECT_NAME} ${SOURCES} "src/Physics/Rigidbody.cpp" "src/Physics/PhysicsWorld.cpp")

# --- Include Dizinleri ---
# src klasörünü include path'e ekle (headerları rahat bulmak için)
target_include_directories(${PROJECT_NAME} PRIVATE src)

# --- Kütüphaneleri Linkle ---
# SFML modüllerini projene bağlıyoruz
target_link_libraries(${PROJECT_NAME} PRIVATE sfml-graphics sfml-window sfml-system)

# (Opsiyonel) Windows'ta konsol penceresi açılmasın istiyorsan:
# if(WIN32)
#     set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "/SUBSYSTEM:WINDOWS")
# endif()