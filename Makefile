# =================================================
# 1. 交叉编译器配置
# =================================================
CROSS_COMPILE = /home/why/rk3588/ELF2-linux-source/prebuilts/gcc/linux-x86/aarch64/gcc-arm-10.3-2021.07-x86_64-aarch64-none-linux-gnu/bin/aarch64-none-linux-gnu-
CC = $(CROSS_COMPILE)gcc
CXX = $(CROSS_COMPILE)g++

# =================================================
# 2. 路径定义
# =================================================
ROOT_DIR = $(CURDIR)

# 基础依赖路径
FFMPEG_INC = /home/why/rk3588/include
RKNN_INC   = /home/why/rk3588/ELF2-linux-source/external/rknpu2/runtime/Linux/librknn_api/include
RGA_INC    = /home/why/rk3588/ELF2-linux-source/external/linux-rga/include
IM2D_INC   = /home/why/rk3588/ELF2-linux-source/external/linux-rga/im2d_api

# OpenCV 路径
OPENCV_DIR = "/home/why/rk3588/rkproject/ai训练/rknn_model_zoo/3rdparty/opencv/opencv-linux-aarch64"
OPENCV_INC = $(OPENCV_DIR)/include
OPENCV_LIB = $(OPENCV_DIR)/lib

# 系统库路径
SYSROOT_LIB = $(ROOT_DIR)/third_party/elf2_sysroot/usr/lib/aarch64-linux-gnu
SYSROOT_INC = $(ROOT_DIR)/third_party/elf2_sysroot/usr/include
SYSROOT_ARCH_INC = $(ROOT_DIR)/third_party/elf2_sysroot/usr/include/aarch64-linux-gnu

# 硬件加速库路径
RKNN_LIB_DIR = /home/why/rk3588/ELF2-linux-source/external/rknpu2/runtime/Linux/librknn_api/aarch64
RGA_LIB_DIR  = "/home/why/rk3588/rkproject/ai训练/rknn_model_zoo/3rdparty/librga/Linux/aarch64"
DRM_LIB_DIR  = /home/why/rk3588/ELF2-linux-source/external/linux-rga/samples/utils/3rdparty/libdrm/lib/arm64
CUSTOM_LIB   = /home/why/rk3588/lib

# =================================================
# 3. 编译选项 (已新增 FreeType 头文件路径)
# =================================================
INC_FLAGS = -I$(FFMPEG_INC) \
            -I$(RKNN_INC) \
            -I$(RGA_INC) \
            -I$(IM2D_INC) \
            -I$(OPENCV_INC) \
            -I$(SYSROOT_INC) \
            -I$(SYSROOT_INC)/freetype2 \
            -I$(SYSROOT_ARCH_INC) \
            -I.

CFLAGS   = -Wall -O2 -g $(INC_FLAGS) -DDISABLE_LIBJPEG -Wl,--allow-shlib-undefined
CXXFLAGS = $(CFLAGS) -std=c++11

# =================================================
# 4. 链接选项 (已新增 -lfreetype)
# =================================================
LDFLAGS = -L$(CUSTOM_LIB) \
          -L$(RKNN_LIB_DIR) \
          -L$(RGA_LIB_DIR) \
          -L$(DRM_LIB_DIR) \
          -L$(OPENCV_LIB) \
          -L$(SYSROOT_LIB) \
          -Wl,-rpath=/usr/lib/aarch64-linux-gnu \
          -l:libavformat.so.60 \
          -l:libavcodec.so.60 \
          -l:libswscale.so.7 \
          -l:libswresample.so.4 \
          -l:libavutil.so.58 \
          -lpaho-mqtt3c \
          -lmodbus \
          -lcurl \
          -lsqlite3 \
          -ldrm \
          -lrga \
          -lrknnrt \
          -lfreetype \
          -lopencv_imgproc -lopencv_core \
          -lm -lpthread -ldl

# =================================================
# 5. 源文件配置 (已新增 osd_cache.c 和 audio_alert.c)
# =================================================
TARGET = main

# 纯 C 源文件
C_SRCS = main.c camera.c ipc.c encoder.c aliyun_mqtt.c zone_detector.c \
         sensor_modbus.c local_store.c video_store.c \ video_uploader.c osd_cache.c audio_alert.c \
         safety_interlock_client.c

# C++ 源文件
CXX_SRCS = postprocess.cc rknn_worker.cc

# 生成对应的 .o 文件列表
OBJS = $(C_SRCS:.c=.o) $(CXX_SRCS:.cc=.o)

# =================================================
# 6. 编译规则
# =================================================
all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

%.o: %.cc
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET)

.PHONY: all clean
