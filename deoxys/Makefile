
#acceptable buildTypes: Release/Debug/Profile

# STOP CHANGING THIS AND COMMITTING IT
# If you want to change the build type then
# either run export buildType=Debug
# or run buildType=Debug make

build_deoxys ?= 0
build_franka ?=0

buildType ?= Release
# CC ?= $(which clang-7)
# CXX ?= $(which clang++-7)
CC = gcc
CXX = g++

ifeq ($(buildType),Debug)
	buildDir=build_debug
else
	buildDir=build
endif


ifeq ($(buildType), RelWithDebInfo)
	cmakeBuildType=$(buildType)
else
	cmakeBuildType=$(buildType)
endif


# get the current make file path and directory
# see http://stackoverflow.com/questions/18136918/how-to-get-current-relative-directory-of-your-makefile#18137056
mkfile_path := $(abspath $(lastword $(MAKEFILE_LIST)))
current_dir := $(patsubst %/,%,$(dir $(mkfile_path)))

.PHONY: all clean cleanup_cache clean_all cleanlogs test build install cmake

all: install

build: cmake 
	$(MAKE) --no-print-directory -C $(buildDir)

clean:
	$(MAKE) --no-print-directory -C $(buildDir) clean

cleanup_cache:
	rm -f $(buildDir)/CMakeCache.txt

clean_all: clean cleanup_cache

cleanlogs:
	(cd logs; rm -rf *.txt *.vraw *.?log *.log *.idx html/*.html)

cmake: CMakeLists.txt
	mkdir -p $(buildDir) && cd $(buildDir) && cmake -DCMAKE_BUILD_TYPE=$(cmakeBuildType) -DCMAKE_INSTALL_PREFIX=$(current_dir) -DCMAKE_LIBRARY_OUTPUT_DIRECTORY=$(current_dir)  -DBUILD_DEOXYS=$(build_deoxys) -DBUILD_FRANKA=$(build_franka)  ..

docs: cmake
	$(MAKE) --no-print-directory -C $(buildDir) docs

install: build
	$(MAKE) --no-print-directory -C $(buildDir) install

