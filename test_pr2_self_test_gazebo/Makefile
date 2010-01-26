# Temporary hack to shorten link lines by turning off CMake's rpath
# generation when running under Hudson (which we notice via the WORKSPACE
# env var).
ifneq ($(strip $(WORKSPACE)),)
EXTRA_CMAKE_FLAGS = -DCMAKE_SKIP_BUILD_RPATH=1
endif
include $(shell rospack find mk)/cmake.mk
