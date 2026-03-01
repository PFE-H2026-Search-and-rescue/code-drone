##############################################################################
#  Copyright 2020 ModalAI Inc.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#  1. Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.
#
#  2. Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation
#     and/or other materials provided with the distribution.
#
#  3. Neither the name of the copyright holder nor the names of its contributors
#     may be used to endorse or promote products derived from this software
#     without specific prior written permission.
#
#  4. The Software is used solely in conjunction with devices provided by
#     ModalAI Inc.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
#  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
##############################################################################

LOCAL_PATH:= $(call my-dir)
VOXL_PORTAL_ROOT_REL:= ../..
VOXL_PORTAL_ROOT_ABS:= $(LOCAL_PATH)/../..

# libusb

include $(CLEAR_VARS)

VOXL_PORTAL_ROOT_REL:= ../..
VOXL_PORTAL_ROOT_ABS:= $(LOCAL_PATH)/../..

LOCAL_SRC_FILES := \
  $(VOXL_PORTAL_ROOT_ABS)/src/main.cpp \
  $(VOXL_PORTAL_ROOT_ABS)/src/page_manager.cpp \
  $(VOXL_PORTAL_ROOT_ABS)/src/video_manager.cpp \
  $(VOXL_PORTAL_ROOT_ABS)/src/imu_manager.cpp \
  $(VOXL_PORTAL_ROOT_ABS)/src/pose_manager.cpp \
  $(VOXL_PORTAL_ROOT_ABS)/src/gps_manager.cpp \
  $(VOXL_PORTAL_ROOT_ABS)/src/plan_manager.cpp \
  $(VOXL_PORTAL_ROOT_ABS)/src/battery_manager.cpp \
  $(VOXL_PORTAL_ROOT_ABS)/src/cpu_manager.cpp \
  $(VOXL_PORTAL_ROOT_ABS)/src/costmap_manager.cpp \
  $(VOXL_PORTAL_ROOT_ABS)/src/pointcloud_manager.cpp \
  $(VOXL_PORTAL_ROOT_ABS)/src/mapper_ptcloud_manager.cpp \
  $(VOXL_PORTAL_ROOT_ABS)/mongoose/mongoose.c \
  $(VOXL_PORTAL_ROOT_ABS)/android/jni/voxl_cutils.c 

LOCAL_C_INCLUDES += \
  $(VOXL_PORTAL_ROOT_ABS)/mongoose \
  $(VOXL_PORTAL_ROOT_ABS)/libjpeg-turbo \
  $(VOXL_PORTAL_ROOT_ABS)/android/jni/include \
  $(VOXL_PORTAL_ROOT_ABS)/../android_build/include \
  $(VOXL_PORTAL_ROOT_ABS)/../../android_build/include 


LOCAL_EXPORT_C_INCLUDES := \
  $(VOXL_PORTAL_ROOT_ABS)/include


LOCAL_LDLIBS :=  \
  -L$(VOXL_PORTAL_ROOT_ABS)/../android_build/lib/ \
  -L$(VOXL_PORTAL_ROOT_ABS)/../../android_build/lib/ \
  -lc++_shared -llog -lturbojpeg -lmodal_pipe -lmodal_json

LOCAL_MODULE := voxl-portal

include $(BUILD_EXECUTABLE)



