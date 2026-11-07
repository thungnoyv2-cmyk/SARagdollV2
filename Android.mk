LOCAL_PATH := $(call my-dir)

# ---- Prebuilt Bullet (build these yourself for armeabi-v7a, see README) ----
include $(CLEAR_VARS)
LOCAL_MODULE := bt_dynamics
LOCAL_SRC_FILES := ../libs_bullet/armeabi-v7a/libBulletDynamics.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := bt_collision
LOCAL_SRC_FILES := ../libs_bullet/armeabi-v7a/libBulletCollision.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := bt_linear
LOCAL_SRC_FILES := ../libs_bullet/armeabi-v7a/libLinearMath.a
include $(PREBUILT_STATIC_LIBRARY)

# ---- The mod ----
include $(CLEAR_VARS)
LOCAL_MODULE    := SARagdoll
LOCAL_SRC_FILES := main.cpp ragdoll.cpp
LOCAL_C_INCLUDES := $(LOCAL_PATH)/aml_headers $(LOCAL_PATH)/../bullet_src/src
LOCAL_CPPFLAGS  := -std=c++17 -O2 -fno-exceptions -fno-rtti -DBT_USE_NEON=0
LOCAL_LDLIBS    := -llog
LOCAL_STATIC_LIBRARIES := bt_dynamics bt_collision bt_linear
include $(BUILD_SHARED_LIBRARY)
