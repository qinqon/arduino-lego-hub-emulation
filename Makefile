FQBN ?= esp32:esp32:dfrobot_romeo_esp32s3

PORT ?= /dev/ttyACM0

SKETCH_PATH = LegoHubEmulation.ino
BUILD_PATH = build

# --- Targets ---

.PHONY: all compile upload clean

all: compile

compile:
	@echo "Compiling $(SKETCH_PATH)..."
	arduino-cli compile --fqbn $(FQBN) --build-path $(BUILD_PATH) $(SKETCH_PATH)
	@echo "Compilation complete."

upload: compile
	@echo "Uploading $(SKETCH_PATH) to $(PORT)..."
	arduino-cli upload -p $(PORT) --fqbn $(FQBN) --build-path $(BUILD_PATH) $(SKETCH_PATH)
	@echo "Upload complete."

monitor: upload
	arduino-cli monitor -p $(PORT) --fqbn $(FQBN)

clean:
	@echo "Cleaning build directory..."
	rm -rf $(BUILD_PATH)
	@echo "Clean complete."

help:
	@echo "Makefile for Arduino project:"
	@echo "  make compile  - Compiles the Arduino sketch."
	@echo "  make upload   - Compiles and uploads the Arduino sketch to the specified port."
	@echo "  make clean    - Removes the build directory."
	@echo ""
	@echo "Configuration:"
	@echo "  Set FQBN and PORT variables. Example:"
	@echo "  make FQBN=arduino:avr:uno PORT=/dev/ttyACM0 upload"
	@echo "  Or modify the Makefile directly."
