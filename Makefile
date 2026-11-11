.DEFAULT_GOAL := build

.PHONY: configure build run test clean

configure:
	cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug

build: configure
	cmake --build build

run: build
	./build/revolute

test: build
	ctest --test-dir build --output-on-failure

clean:
	cmake --build build --target clean
