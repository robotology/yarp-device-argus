/*
 * Copyright (C) 2006-2024 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include <yarp/os/LogComponent.h>
#include <yarp/os/Value.h>
#include <yarp/sig/ImageUtils.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#ifdef USE_CUDA
#include <opencv2/cudawarping.hpp>
#include <opencv2/cudaimgproc.hpp>
#endif  // USE_CUDA

#include "argusCameraDriver.h"

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;

using namespace std;
using namespace Argus;
using namespace EGLStream;

// VERY IMPORTANT ABOUT WHITE BALANCE: the YARP interfaces cannot allow to set a feature with
// 3 values, 2 is maximum and until now we always used blue and red in this order. Then we ignore
// green

static const std::vector<cameraFeature_id_t> supported_features{YARP_FEATURE_EXPOSURE, YARP_FEATURE_SATURATION, YARP_FEATURE_SHARPNESS, YARP_FEATURE_WHITE_BALANCE,
                                                                YARP_FEATURE_FRAME_RATE};

static const std::vector<cameraFeature_id_t> features_with_auto{YARP_FEATURE_EXPOSURE, YARP_FEATURE_WHITE_BALANCE};

static const std::map<cameraFeature_id_t, std::pair<double, double>> featureMinMax{{YARP_FEATURE_EXPOSURE, {-2.0, 2.0}}, 
                                                                                   {YARP_FEATURE_SATURATION, {0.0, 2.0}}, 
                                                                                   {YARP_FEATURE_SHARPNESS, {-1.0, 1.0}}, 
                                                                                   {YARP_FEATURE_WHITE_BALANCE, {1.0, 8.0}}, // not sure about it, the doc is not clear, found empirically
                                                                                   {YARP_FEATURE_GAIN, {1.0, 3981.07}}}; 

static const std::map<double, NV::Rotation> rotationToNVRot{{0.0, NV::ROTATION_0}, {90.0, NV::ROTATION_90}, {-90.0, NV::ROTATION_270}, {180.0, NV::ROTATION_180}};
static const std::map<double, double> rotationToCVRot{{0.0, 0.0}, {90.0, cv::ROTATE_90_COUNTERCLOCKWISE}, {-90.0, cv::ROTATE_90_CLOCKWISE}, {180.0, cv::ROTATE_180}};

static const std::map<std::string, std::vector<Argus::Size2D<uint32_t>>> cameraResolutions{
    {"imx415", {Size2D<uint32_t>(1280, 720), Size2D<uint32_t>(1920, 1080), Size2D<uint32_t>(3840, 2160)}},
    {"imx678", {Size2D<uint32_t>(3840, 2160), Size2D<uint32_t>(2560, 1440), Size2D<uint32_t>(1920, 1080)}}
};

// We usually set the features through a range between 0 an 1, we have to translate it in meaninful value for the camera
double fromZeroOneToRange(cameraFeature_id_t feature, double value)
{
    return value * (featureMinMax.at(feature).second - featureMinMax.at(feature).first) + featureMinMax.at(feature).first;
}

// We want the features in the range 0 1
double fromRangeToZeroOne(cameraFeature_id_t feature, double value)
{
    return (value - featureMinMax.at(feature).first) / (featureMinMax.at(feature).second - featureMinMax.at(feature).first);
}

bool argusCameraDriver::setFramerate(const uint64_t _fps)
{
    IRequest *iRequest = interface_cast<IRequest>(m_request);
    IAutoControlSettings *m_iAutoControlSettings = interface_cast<IAutoControlSettings>(iRequest->getAutoControlSettings());
    ISourceSettings *m_iSourceSettings = interface_cast<ISourceSettings>(iRequest->getSourceSettings());

    m_iAutoControlSettings->setAeLock(true);

    // According to https://docs.nvidia.com/jetson/l4t-multimedia/classArgus_1_1ISourceSettings.html, frame duration range is expressed in nanoseconds
    uint64_t frameDuration = 1e9 / _fps;
    bool ret = true;
    ret = ret && m_iSourceSettings->setFrameDurationRange(Argus::Range<uint64_t>(frameDuration)) == STATUS_OK;
    ret = ret && m_iSourceSettings->setExposureTimeRange(Argus::Range<uint64_t>(frameDuration)) == STATUS_OK;
    if (ret)
    {
        m_fps = _fps;
    }
    else
    {
        yCError(ARGUS_CAMERA) << "The required frame rate" << m_fps << "cannot be set";
        return false;
    }

    return ret;
}

bool parseUint32Param(std::string param_name, std::uint32_t& param, yarp::os::Searchable& config)
{
    if (config.check(param_name) && config.find(param_name).isInt32())
    {
        param = config.find(param_name).asInt32();
        return true;
    }
    else
    {
        yCWarning(ARGUS_CAMERA) << param_name << "parameter not specified, using" << param;
        return false;
    }
}

bool argusCameraDriver::startCamera()
{
    setFramerate(m_fps);
    ICaptureSession *iCaptureSession = interface_cast<ICaptureSession>(m_captureSession);
    if (m_consumer)
    {
        if (!iCaptureSession->isRepeating())
        {
            iCaptureSession->repeat(m_request.get());
        }
    }
    return true;
}

bool argusCameraDriver::stopCamera()
{
    ICaptureSession *iCaptureSession = interface_cast<ICaptureSession>(m_captureSession);
    if (m_consumer)
    {
        if (iCaptureSession->isRepeating())
        {
            iCaptureSession->stopRepeat();
            iCaptureSession->waitForIdle();
        }
    }
    return true;
}

bool argusCameraDriver::open(Searchable& config)
{
    bool ok{true};
    yCDebug(ARGUS_CAMERA) << "input params are " << config.toString();

    if(!parseParams(config)) {
        yCError(ARGUS_CAMERA) << "Error parsing parameters";
        return false;
    }

    if (m_period != 0.0)
    {
        m_fps = 1.0 / m_period;
    }

    // FIXME handle m_period = 0.0

    m_cameraProvider.reset(CameraProvider::create());
    ICameraProvider *iCameraProvider = interface_cast<ICameraProvider>(m_cameraProvider);
    if (!iCameraProvider)
    {
        yCError(ARGUS_CAMERA) << "Failed to create CameraProvider";
        return false;
    }

    /* Get the camera devices */
    iCameraProvider->getCameraDevices(&m_cameraDevices);
    if (m_cameraDevices.size() == 0)
    {
        yCError(ARGUS_CAMERA) << "No cameras available";
        return false;
    }

    ICameraProperties *iCameraProperties = interface_cast<ICameraProperties>(m_cameraDevices[m_d]);
    if (!iCameraProperties)
    {
        yCError(ARGUS_CAMERA) << "Failed to get ICameraProperties interface";
        return false;
    }
    
    if (m_d >= m_cameraDevices.size())
    {
        yCError(ARGUS_CAMERA) << "Camera device index d =" << m_d << "is invalid.";
        return false;
    }

    /* Create the capture session using the first device and get the core interface */
    m_captureSession.reset(iCameraProvider->createCaptureSession(m_cameraDevices[m_d]));
    ICaptureSession *iCaptureSession = interface_cast<ICaptureSession>(m_captureSession);
    if (!iCaptureSession)
    {
        yCError(ARGUS_CAMERA) << "Failed to get ICaptureSession interface";
        return false;
    }

    m_streamSettings.reset(iCaptureSession->createOutputStreamSettings(STREAM_TYPE_EGL));
    IEGLOutputStreamSettings *iEglStreamSettings = interface_cast<IEGLOutputStreamSettings>(m_streamSettings);
    if (!iEglStreamSettings)
    {
        yCError(ARGUS_CAMERA) << "Failed to get IEGLOutputStreamSettings interface";
        return false;
    }

    ok = ok && setRgbResolution(m_width, m_height);

    #ifdef USE_CUDA
        yCDebug(ARGUS_CAMERA) << "Using CUDA!";
        gpu_rgba_img = cv::cuda::GpuMat(m_width, m_height, CV_8UC4);
        gpu_bgr_img = cv::cuda::GpuMat(m_width, m_height, CV_8UC3);
    #else
        yCDebug(ARGUS_CAMERA) << "Not using CUDA!";
    #endif

    bgr_img = cv::Mat(m_height, m_width, CV_8UC3);
    rgba_img = cv::Mat(m_height, m_width, CV_8UC4);

    return ok && startCamera();
}

bool argusCameraDriver::close()
{
    return true;
}

int argusCameraDriver::getRgbHeight()
{
    return m_height;
}

int argusCameraDriver::getRgbWidth()
{
    return m_width;
}

bool argusCameraDriver::getRgbSupportedConfigurations(yarp::sig::VectorOf<CameraConfig>& configurations)
{
    yCWarning(ARGUS_CAMERA) << "getRgbSupportedConfigurations not implemented yet";
    return false;
}

bool argusCameraDriver::getRgbResolution(int& width, int& height)
{
    width = m_width;
    height = m_height;
    return true;
}

bool argusCameraDriver::setRgbResolution(int width, int height)
{
    stopCamera();

    IEGLOutputStreamSettings *iEglStreamSettings = interface_cast<IEGLOutputStreamSettings>(m_streamSettings);
    ICameraProperties *iCameraProperties = interface_cast<ICameraProperties>(m_cameraDevices[m_d]);
    ICaptureSession *iCaptureSession = interface_cast<ICaptureSession>(m_captureSession);
    if (!iCaptureSession)
    {
        yCError(ARGUS_CAMERA) << "Failed to get ICaptureSession interface";
        return false;
    }

    if (width > 0 && height > 0)
    {
        int nearestWidth = -1;
        int nearestHeight = -1;
        double minDistance = std::numeric_limits<double>::max();

        auto supportedResolutions = cameraResolutions.at(iCameraProperties->getModelName());
        for (auto &resolution : supportedResolutions)
        {
            if (resolution.width() == width && resolution.height() == height)
                {
                    yCDebug(ARGUS_CAMERA) << "The resolution" << resolution.width() << "x" << resolution.height() << "is available";
                    nearestWidth = width;
                    nearestHeight = height;
                    break;
                }
                else
                {
                    yCWarning(ARGUS_CAMERA) << "The set width and height are different from the available ones. Searching for the nearest resolution...";
                    double distance = std::abs(int(resolution.width() - width)) + std::abs(int(resolution.height() - height));
                    if (distance < minDistance)
                    {
                        minDistance = distance;
                        nearestWidth = resolution.width();
                        nearestHeight = resolution.height();
                    }
                }
        }

        if (nearestWidth != -1 && nearestHeight != -1)
        {
            yCInfo(ARGUS_CAMERA) << "Nearest resolution found:" << nearestWidth << "x" << nearestHeight;
        }

        if (m_rotation_with_crop)
        {
            if (m_rotation == -90.0 || m_rotation == 90.0)
            {
                std::swap(width, height);
            }
        }

        Size2D<uint32_t> resolution{nearestWidth, nearestHeight};
        iEglStreamSettings->setPixelFormat(PIXEL_FMT_YCbCr_420_888);
        iEglStreamSettings->setResolution(resolution);

        if(iEglStreamSettings->setResolution(Size2D<uint32_t>(resolution)) == STATUS_OK)
        {
            m_width = width;
            m_height = height;
        }
    }

    m_stream.reset(iCaptureSession->createOutputStream(m_streamSettings.get()));
    m_consumer.reset(FrameConsumer::create(m_stream.get()));

    if (!m_consumer)
    {
        yCError(ARGUS_CAMERA) << "Failed to create FrameConsumer";
        return false;
    }

    m_request.reset(iCaptureSession->createRequest(Argus::CAPTURE_INTENT_PREVIEW));
    Argus::IRequest *iRequest = Argus::interface_cast<Argus::IRequest>(m_request);
    if (iRequest->enableOutputStream(m_stream.get()) != STATUS_OK)
    {
        yCError(ARGUS_CAMERA) << "Failed to enable output stream";
        return false;
    }

    startCamera();
    return true;
}

bool argusCameraDriver::setRgbFOV(double horizontalFov, double verticalFov)
{
    yCWarning(ARGUS_CAMERA) << "setRgbFOV not supported";
    return false;
}

bool argusCameraDriver::getRgbFOV(double& horizontalFov, double& verticalFov)
{
    yCWarning(ARGUS_CAMERA) << "getRgbFOV not supported";
    return false;
}

bool argusCameraDriver::getRgbMirroring(bool& mirror)
{
    yCWarning(ARGUS_CAMERA) << "Mirroring not supported";
    return false;
}

bool argusCameraDriver::setRgbMirroring(bool mirror)
{
    yCWarning(ARGUS_CAMERA) << "Mirroring not supported";
    return false;
}

bool argusCameraDriver::getRgbIntrinsicParam(Property& intrinsic)
{
    yCWarning(ARGUS_CAMERA) << "getRgbIntrinsicParam not supported"; //no intrinsic parameters stored in the eeprom of the camera
    return false;
}

bool argusCameraDriver::getCameraDescription(CameraDescriptor* camera)
{
    ICameraProperties *iCameraProperties = interface_cast<ICameraProperties>(m_cameraDevices[m_d]);
    camera->deviceDescription = iCameraProperties->getModelName();
    camera->busType = BUS_UNKNOWN;
    return true;
}

bool argusCameraDriver::hasFeature(int feature, bool* hasFeature)
{
    cameraFeature_id_t f;
    f = static_cast<cameraFeature_id_t>(feature);
    if (f < YARP_FEATURE_BRIGHTNESS || f > YARP_FEATURE_NUMBER_OF - 1)
    {
        return false;
    }

    *hasFeature = std::find(supported_features.begin(), supported_features.end(), f) != supported_features.end();

    return true;
}

bool argusCameraDriver::setFeature(int feature, double value)
{
    bool b = false;
    if (!hasFeature(feature, &b) || !b)
    {
        yCError(ARGUS_CAMERA) << "Feature not supported!";
        return false;
    }
    b = false;
    auto f = static_cast<cameraFeature_id_t>(feature);

    Argus::IRequest *iRequest = Argus::interface_cast<Argus::IRequest>(m_request);
    IAutoControlSettings *m_iAutoControlSettings = interface_cast<IAutoControlSettings>(iRequest->getAutoControlSettings());
    IEdgeEnhanceSettings *m_iEdgeEnhanceSettings = interface_cast<IEdgeEnhanceSettings>(m_request);
    stopCamera();

    switch (f)
    {
        case YARP_FEATURE_EXPOSURE:
            m_iAutoControlSettings->setExposureCompensation(fromZeroOneToRange(f, value));
            b = true;
            break;
        case YARP_FEATURE_SATURATION:
            m_iAutoControlSettings->setColorSaturation(fromZeroOneToRange(f, value));
            b = true;
            break;
        case YARP_FEATURE_SHARPNESS:
            m_iEdgeEnhanceSettings->setEdgeEnhanceMode(EDGE_ENHANCE_MODE_HIGH_QUALITY);
            m_iEdgeEnhanceSettings->setEdgeEnhanceStrength(fromZeroOneToRange(f, value));
            b = true;
            break;
        case YARP_FEATURE_WHITE_BALANCE:
            b = false;
            yCError(ARGUS_CAMERA) << "White balance require 2 values";
        case YARP_FEATURE_FRAME_RATE:
            b = setFramerate(value);
            break;
        default:
            yCError(ARGUS_CAMERA) << "Feature not supported!";
            return false;
    }

    startCamera();
    return b;
}

bool argusCameraDriver::getFeature(int feature, double* value)
{
    bool b = false;
    if (!hasFeature(feature, &b) || !b)
    {
        yCError(ARGUS_CAMERA) << "Feature not supported!";
        return false;
    }
    b = false;
    auto f = static_cast<cameraFeature_id_t>(feature);

    Argus::IRequest *iRequest = Argus::interface_cast<Argus::IRequest>(m_request);
    IAutoControlSettings *m_iAutoControlSettings = interface_cast<IAutoControlSettings>(iRequest->getAutoControlSettings());
    IEdgeEnhanceSettings *m_iEdgeEnhanceSettings = interface_cast<IEdgeEnhanceSettings>(m_request);

    switch (f)
    {
        case YARP_FEATURE_EXPOSURE:
            *value = m_iAutoControlSettings->getExposureCompensation();
            b = true;
            break;
        case YARP_FEATURE_SATURATION:
            m_iAutoControlSettings->setColorSaturationEnable(true);
            *value = m_iAutoControlSettings->getColorSaturation();
            b = true;
            break;
        case YARP_FEATURE_SHARPNESS:
            *value = m_iEdgeEnhanceSettings->getEdgeEnhanceStrength();
            b = true;
            break;
        case YARP_FEATURE_WHITE_BALANCE:
            b = false;
            yCError(ARGUS_CAMERA) << "White balance is a 2-values feature";
            break;
        case YARP_FEATURE_FRAME_RATE:
            b = true;
            *value = m_fps;
            break;
        default:
            yCError(ARGUS_CAMERA) << "Feature not supported!";
            return false;
    }

    *value = fromRangeToZeroOne(f, *value);
    yCDebug(ARGUS_CAMERA) << "In 0-1" << *value;
    return b;
}

bool argusCameraDriver::setFeature(int feature, double value1, double value2)
{
    auto f = static_cast<cameraFeature_id_t>(feature);
    auto res = true;
    Argus::IRequest *iRequest = Argus::interface_cast<Argus::IRequest>(m_request);
    IAutoControlSettings *m_iAutoControlSettings = interface_cast<IAutoControlSettings>(iRequest->getAutoControlSettings());
    stopCamera();

    if (f != YARP_FEATURE_WHITE_BALANCE)
    {
        yCError(ARGUS_CAMERA) << YARP_FEATURE_WHITE_BALANCE << "is not a 2-values feature supported";
        return false;
    }

    m_iAutoControlSettings->setAeLock(true);
    m_iAutoControlSettings->setAwbLock(false);
    m_iAutoControlSettings->setAwbMode(AWB_MODE_MANUAL);
    BayerTuple<float> wbGains(fromZeroOneToRange(f, value2), fromZeroOneToRange(f, 0.0), fromZeroOneToRange(f, 0.0), fromZeroOneToRange(f, value1));
    m_iAutoControlSettings->setWbGains(wbGains);

    startCamera();
    return res;
}

bool argusCameraDriver::getFeature(int feature, double* value1, double* value2)
{
    auto f = static_cast<cameraFeature_id_t>(feature);
    auto res = true;

    Argus::IRequest *iRequest = Argus::interface_cast<Argus::IRequest>(m_request);
    IAutoControlSettings *m_iAutoControlSettings = interface_cast<IAutoControlSettings>(iRequest->getAutoControlSettings());

    if (f != YARP_FEATURE_WHITE_BALANCE)
    {
        yCError(ARGUS_CAMERA) << "This is not a 2-values feature supported";
        return false;
    }

    *value1 = fromRangeToZeroOne(f, m_iAutoControlSettings->getWbGains().r());
    *value2 = fromRangeToZeroOne(f, m_iAutoControlSettings->getWbGains().b());
    return res;
}

bool argusCameraDriver::hasOnOff(int feature, bool* HasOnOff)
{
    return hasAuto(feature, HasOnOff);
}

bool argusCameraDriver::setActive(int feature, bool onoff)
{
    bool b = false;
    auto f = static_cast<cameraFeature_id_t>(feature);
    Argus::IRequest *iRequest = Argus::interface_cast<Argus::IRequest>(m_request);
    IAutoControlSettings *m_iAutoControlSettings = interface_cast<IAutoControlSettings>(iRequest->getAutoControlSettings());
    stopCamera();

    if (!hasFeature(feature, &b) || !b)
    {
        yCError(ARGUS_CAMERA) << "Feature" << feature << "not supported!";
        return false;
    }

    if (!hasOnOff(feature, &b) || !b)
    {
        yCError(ARGUS_CAMERA) << "Feature" << feature << "does not have OnOff.. call hasOnOff() to know if a specific feature support OnOff mode";
        return false;
    }

    switch (f)
    {
        case YARP_FEATURE_EXPOSURE:
            m_iAutoControlSettings->setAeLock(!onoff);
            b = true;
            break;
        case YARP_FEATURE_WHITE_BALANCE:
            m_iAutoControlSettings->setAwbMode(AWB_MODE_AUTO);
            m_iAutoControlSettings->setAwbLock(!onoff);
            b = true;
            break;
        default:
            yCError(ARGUS_CAMERA) << "Feature" << feature << "not supported!";
            return false;
    }

    startCamera();
    return b;
}

bool argusCameraDriver::getActive(int feature, bool* isActive)
{
    bool b = false;
    auto f = static_cast<cameraFeature_id_t>(feature);
    Argus::IRequest *iRequest = Argus::interface_cast<Argus::IRequest>(m_request);
    IAutoControlSettings *m_iAutoControlSettings = interface_cast<IAutoControlSettings>(iRequest->getAutoControlSettings());
    if (!hasFeature(feature, &b) || !b)
    {
        yCError(ARGUS_CAMERA) << "Feature" << feature << "not supported!";
        return false;
    }

    if (!hasOnOff(feature, &b) || !b)
    {
        yCError(ARGUS_CAMERA) << "Feature" << feature << "does not have OnOff.. call hasOnOff() to know if a specific feature support OnOff mode";
        return false;
    }

    bool val_to_get;
    switch (f)
    {
        case YARP_FEATURE_EXPOSURE:
            val_to_get = !(m_iAutoControlSettings->getAeLock());
            b = true;
            break;
        case YARP_FEATURE_WHITE_BALANCE:
            val_to_get = !(m_iAutoControlSettings->getAwbLock());
            b = true;
            break;
        default:
            yCError(ARGUS_CAMERA) << "Feature" << feature << "not supported!";
            return false;
    }

    if (b)
    {
        if (val_to_get)
        {
            *isActive = true;
        }
        else
        {
            *isActive = false;
        }
    }
    return b;
}

bool argusCameraDriver::hasAuto(int feature, bool* hasAuto)
{
    cameraFeature_id_t f;
    f = static_cast<cameraFeature_id_t>(feature);
    if (f < YARP_FEATURE_BRIGHTNESS || f > YARP_FEATURE_NUMBER_OF - 1)
    {
        return false;
    }

    *hasAuto = std::find(features_with_auto.begin(), features_with_auto.end(), f) != features_with_auto.end();

    return true;
}

bool argusCameraDriver::hasManual(int feature, bool* hasManual)
{
    return hasFeature(feature, hasManual);
}

bool argusCameraDriver::hasOnePush(int feature, bool* hasOnePush)
{
    return hasAuto(feature, hasOnePush);
}

bool argusCameraDriver::setMode(int feature, FeatureMode mode)
{
    bool b{false};
    if (!hasAuto(feature, &b) || !b)
    {
        yCError(ARGUS_CAMERA) << "Feature" << feature << "not supported!";
        return false;
    }

    switch (mode)
    {
        case MODE_AUTO:
            return setActive(feature, true);
        case MODE_MANUAL:
            return setActive(feature, false);
        case MODE_UNKNOWN:
            return false;
        default:
            return false;
    }
    return b;
}

bool argusCameraDriver::getMode(int feature, FeatureMode* mode)
{
    bool b{false};
    if (!hasAuto(feature, &b) || !b)
    {
        yCError(ARGUS_CAMERA) << "Feature" << feature << "not supported!";
        return false;
    }
    bool get_active{false};
    b = b && getActive(feature, &get_active);

    if (b)
    {
        if (get_active)
        {
            *mode = MODE_AUTO;
        }
        else
        {
            *mode = MODE_MANUAL;
        }
    }
    return b;
}

bool argusCameraDriver::setOnePush(int feature)
{
    bool b = false;
    if (!hasOnePush(feature, &b) || !b)
    {
        yCError(ARGUS_CAMERA) << "Feature" << feature << "doesn't have OnePush";
        return false;
    }

    b = b && setMode(feature, MODE_AUTO);
    b = b && setMode(feature, MODE_MANUAL);

    return b;
}

bool argusCameraDriver::getImage(yarp::sig::ImageOf<yarp::sig::PixelRgb>& image)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    NvBufSurface* nvBufSurface = nullptr;

    IFrameConsumer *iFrameConsumer = interface_cast<IFrameConsumer>(m_consumer);
    UniqueObj<Frame> frame(iFrameConsumer->acquireFrame(TIMEOUT_INFINITE));
    IFrame *iFrame = interface_cast<IFrame>(frame);

    if(iFrame)
    {
        auto img = iFrame->getImage();
        auto image2d(Argus::interface_cast<EGLStream::IImage2D>(img));
        auto width = image2d->getSize()[0];
        auto height = image2d->getSize()[1];

        NV::IImageNativeBuffer *iNativeBuffer = interface_cast<NV::IImageNativeBuffer>(img);
        if (!iNativeBuffer)
        {
            yCError(ARGUS_CAMERA) << "IImageNativeBuffer not supported by IImage"; 
        }

        double rotation = 0.0;
        if (m_rotation_with_crop)
        {
            // If m_rotation_with_crop = true, width and height are swapped and the image is stored in a buffer already rotated by m_rotation.
            // In this way, no further transformations need to be done with OpenCV.
            rotation = m_rotation;
        }

        int fd = iNativeBuffer->createNvBuffer(image2d->getSize(), NVBUF_COLOR_FORMAT_RGBA, NVBUF_LAYOUT_PITCH, rotationToNVRot.at(rotation));
        
        if (fd == -1)
        {
            yCError(ARGUS_CAMERA) << "Failed to create NvBuffer";
            return false;
        }

        if (NvBufSurfaceFromFd(fd, (void**)(&nvBufSurface)) == -1)
        {
            yCError(ARGUS_CAMERA) << "Cannot get NvBufSurface from fd";
            return false;
        }

        if (NvBufSurfaceMap(nvBufSurface, 0, 0, NVBUF_MAP_READ) != STATUS_OK)
        {
            yCError(ARGUS_CAMERA) << "Failed to map NvBufSurface";
            return false;
        }

        rgba_img = cv::Mat(height, width, CV_8UC4, nvBufSurface->surfaceList->mappedAddr.addr[0]);
#ifdef USE_CUDA
        gpu_rgba_img.upload(rgba_img);
        cv::cuda::cvtColor(gpu_rgba_img, gpu_bgr_img, cv::COLOR_RGBA2BGR);

        if (!m_rotation_with_crop && m_rotation != 0.0)
        {
            cv::Point2f img_center((gpu_bgr_img.cols - 1) / 2.0, (gpu_bgr_img.rows - 1) / 2.0);
            cv::Mat M = cv::getRotationMatrix2D(img_center, m_rotation, 1.0);
            // Workaround since with cv::cuda::warpAffine, source and dest images CANNOT be the same (otherwise will result in black frames)
            cv::cuda::GpuMat tmp;
            cv::cuda::warpAffine(gpu_bgr_img, tmp, M, gpu_bgr_img.size());
            gpu_bgr_img = std::move(tmp);
        }
        
        if (m_width != width || m_height != height)
        {
            cv::Size size(m_width, m_height);
            cv::cuda::resize(gpu_bgr_img, gpu_bgr_img, size);
        }
        gpu_bgr_img.download(bgr_img);
#else
        cv::cvtColor(rgba_img, bgr_img, cv::COLOR_RGBA2BGR);

        if (!m_rotation_with_crop && m_rotation != 0.0)
        {
            cv::Point2f img_center((bgr_img.cols - 1) / 2.0, (bgr_img.rows - 1) / 2.0);
            cv::Mat M = cv::getRotationMatrix2D(img_center, m_rotation, 1.0);
            cv::warpAffine(bgr_img, bgr_img, M, bgr_img.size());
        }
        
        if (m_width != width || m_height != height)
        {
            cv::Size size(m_width, m_height);
            cv::resize(bgr_img, bgr_img, size);
        }
#endif  // USE_CUDA
        image.copy(yarp::cv::fromCvMat<yarp::sig::PixelRgb>(bgr_img));

        if (NvBufSurfaceUnMap(nvBufSurface, 0, 0) != STATUS_OK) 
        {
            yCError(ARGUS_CAMERA) << "Failed to unmap NvBufSurface";
        }

        if (NvBufSurfaceDestroy(nvBufSurface) != STATUS_OK) 
        {
            yCError(ARGUS_CAMERA) << "Failed to free the NvBufSurface";
        }
    }

    return true;
}

int argusCameraDriver::height() const
{
    return m_height;
}

int argusCameraDriver::width() const
{
    return m_width;
}
