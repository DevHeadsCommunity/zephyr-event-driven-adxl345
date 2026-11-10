cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(adxl345_event_driven)

target_sources(app PRIVATE src/main.c)
target_sources_ifdef(CONFIG_ADXL345_BURST app PRIVATE drivers/adxl345_burst/adxl345_burst.c)
target_include_directories(app PRIVATE drivers/adxl345_burst)
