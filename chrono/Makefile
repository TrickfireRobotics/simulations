BUILD_DIR := ./build
SOURCES := $(wildcard src/*.cpp src/*.hpp)

.PHONY: all run clean

all: $(BUILD_DIR)/sim

$(BUILD_DIR)/sim: ./CMakeLists.txt $(SOURCES)
	cmake -GNinja -B $(BUILD_DIR) -S . -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
	cmake --build $(BUILD_DIR)

run: $(BUILD_DIR)/sim
	cd $(BUILD_DIR) && ./sim

clean:
	rm -rf $(BUILD_DIR)
