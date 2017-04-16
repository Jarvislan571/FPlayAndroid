LOCAL_PATH      := $(call my-dir)
include $(CLEAR_VARS)
LOCAL_MODULE    := MediaContextJni
LOCAL_LDLIBS    := -llog -lOpenSLES -ldl
LOCAL_SRC_FILES := x/MediaContextJni.cpp \
	x/iir/Biquad.cpp \
	x/iir/Butterworth.cpp \
	x/iir/Cascade.cpp \
	x/iir/PoleFilter.cpp \
	x/iir/RootFinder.cpp
LOCAL_CFLAGS := -DNDEBUG -fPIC -ffunction-sections -fdata-sections -Ofast -ftree-vectorize
LOCAL_CPPFLAGS := -DNDEBUG -fPIC -ffunction-sections -fdata-sections -Ofast -ftree-vectorize
ifeq ($(TARGET_ARCH_ABI),armeabi-v7a)
LOCAL_ARM_NEON := true
LOCAL_CFLAGS += -mfpu=neon -march=armv7 -mthumb
LOCAL_CPPFLAGS += -mfpu=neon -march=armv7 -mthumb
LOCAL_SRC_FILES += x/MediaContextJniNeon.cpp.neon
endif
LOCAL_LDFLAGS += -Wl,--gc-sections
include $(BUILD_SHARED_LIBRARY)
include $(CLEAR_VARS)
LOCAL_MODULE    := SimpleVisualizerJni
LOCAL_LDLIBS    := -landroid -ljnigraphics -llog -lGLESv2
LOCAL_SRC_FILES := SimpleVisualizerJni.cpp
ifeq ($(TARGET_ARCH_ABI),armeabi-v7a)
LOCAL_ARM_NEON := true
LOCAL_CFLAGS := -DNDEBUG -fPIC -ffunction-sections -fdata-sections -Ofast -ftree-vectorize
LOCAL_CPPFLAGS := -DNDEBUG -fPIC -ffunction-sections -fdata-sections -Ofast -ftree-vectorize
LOCAL_SRC_FILES += NeonFunctions.cpp.neon
else
LOCAL_SRC_FILES += NeonFunctions.cpp
endif
LOCAL_LDFLAGS += -Wl,--gc-sections
include $(BUILD_SHARED_LIBRARY)