/*
 * Copyright (C) 2006-2024 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef ARGUS_DRIVER_H
#define ARGUS_DRIVER_H

#include <Argus/Argus.h>
#include <Argus/Ext/SyncSensorCalibrationData.h>
#include <EGLStream/EGLStream.h>
#include <EGLStream/NV/ImageNativeBuffer.h>
#include <EGLStream/NV/NvBufSurface.h>

#include "argusCameraDriver_ParamsParser.h"

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IFrameGrabberControls.h>
#include <yarp/dev/IFrameGrabberImage.h>
#include <yarp/dev/IRgbVisualParams.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/all.h>
#include <yarp/cv/Cv.h>

#include <opencv2/opencv.hpp>

#include <cstring>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <typeinfo>

namespace
{
YARP_LOG_COMPONENT(ARGUS_CAMERA, "yarp.device.argusCamera")
}

class argusCameraDriver : public yarp::dev::DeviceDriver,
                          public yarp::dev::IFrameGrabberControls,
                          public yarp::dev::IFrameGrabberImage,
                          public yarp::dev::IRgbVisualParams,
                          public argusCameraDriver_ParamsParser
{
   public:
    argusCameraDriver() = default;
    ~argusCameraDriver() override = default;

    // DeviceDriver
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    // IRgbVisualParams
    int getRgbHeight() override;
    int getRgbWidth() override;
    bool getRgbSupportedConfigurations(yarp::sig::VectorOf<yarp::dev::CameraConfig>& configurations) override;
    bool getRgbResolution(int& width, int& height) override;
    bool setRgbResolution(int width, int height) override;
    bool getRgbFOV(double& horizontalFov, double& verticalFov) override;
    bool setRgbFOV(double horizontalFov, double verticalFov) override;
    bool getRgbMirroring(bool& mirror) override;
    bool setRgbMirroring(bool mirror) override;
    bool getRgbIntrinsicParam(yarp::os::Property& intrinsic) override;

    // IFrameGrabberControls
    bool getCameraDescription(CameraDescriptor* camera) override;
    bool hasFeature(int feature, bool* hasFeature) override;
    bool setFeature(int feature, double value) override;
    bool getFeature(int feature, double* value) override;
    bool setFeature(int feature, double value1, double value2) override;
    bool getFeature(int feature, double* value1, double* value2) override;
    bool hasOnOff(int feature, bool* HasOnOff) override;
    bool setActive(int feature, bool onoff) override;
    bool getActive(int feature, bool* isActive) override;
    bool hasAuto(int feature, bool* hasAuto) override;
    bool hasManual(int feature, bool* hasManual) override;
    bool hasOnePush(int feature, bool* hasOnePush) override;
    bool setMode(int feature, FeatureMode mode) override;
    bool getMode(int feature, FeatureMode* mode) override;
    bool setOnePush(int feature) override;

    // IFrameGrabberImage
    bool getImage(yarp::sig::ImageOf<yarp::sig::PixelRgb>& image) override;
    int height() const override;
    int width() const override;

   private:
    bool setFramerate(const uint64_t _fps);
    bool startCamera();
    bool stopCamera();

    mutable std::mutex m_mutex;
    uint64_t m_fps{90};
    cv::Mat rgba_img, bgr_img;
#ifdef USE_CUDA
    cv::cuda::GpuMat gpu_rgba_img, gpu_bgr_img;
#endif

    Argus::UniqueObj<Argus::CameraProvider> m_cameraProvider;
    Argus::UniqueObj<Argus::OutputStream> m_stream;
    Argus::UniqueObj<Argus::OutputStreamSettings> m_streamSettings;
    Argus::UniqueObj<Argus::Request> m_request;
    Argus::UniqueObj<Argus::CaptureSession> m_captureSession;
    Argus::UniqueObj<EGLStream::FrameConsumer> m_consumer;
    std::vector<Argus::CameraDevice*> m_cameraDevices;
    Argus::ISourceSettings *m_iSourceSettings;
    Argus::IAutoControlSettings *m_iAutoControlSettings;
    Argus::IEdgeEnhanceSettings *m_iEdgeEnhanceSettings;
};
#endif  // ARGUS_DRIVER_H
