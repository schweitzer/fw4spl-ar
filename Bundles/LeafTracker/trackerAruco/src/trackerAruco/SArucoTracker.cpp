/* ***** BEGIN LICENSE BLOCK *****
 * FW4SPL - Copyright (C) IRCAD, 2014-2016.
 * Distributed under the terms of the GNU Lesser General Public License (LGPL) as
 * published by the Free Software Foundation.
 * ****** END LICENSE BLOCK ****** */


#include "trackerAruco/SArucoTracker.hpp"

#define FW_PROFILING_DISABLED
#include <fwCore/Profiling.hpp>

#include <arData/Camera.hpp>
#include <arData/FrameTL.hpp>
#include <arData/MarkerTL.hpp>

#include <fwCom/Signal.hxx>
#include <fwCom/Slots.hxx>
#include <fwData/Composite.hpp>

#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/lexical_cast.hpp>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

//-----------------------------------------------------------------------------

namespace trackerAruco
{
fwServicesRegisterMacro(::tracker::ITracker, ::trackerAruco::SArucoTracker, ::fwData::Composite);
//-----------------------------------------------------------------------------

const ::fwCom::Signals::SignalKeyType SArucoTracker::s_DETECTION_DONE_SIG = "detectionDone";

const ::fwCom::Slots::SlotKeyType SArucoTracker::s_DISPLAY_TAGS_SLOT            = "displayTags";

const ::fwCom::Slots::SlotKeyType SArucoTracker::s_DETECT_MARKER_SLOT = "detectMarker";

const ::fwServices::IService::KeyType s_FRAMETL_INPUT     = "frameTL";
const ::fwServices::IService::KeyType s_CAMERA_INPUT      = "camera";
const ::fwServices::IService::KeyType s_TAGTL_INOUT_GROUP = "tagTL";

//-----------------------------------------------------------------------------

SArucoTracker::SArucoTracker() throw ():
    m_lastTimestamp(0),
    m_isInitialized(false),
    m_debugMarkers(false)
{
    m_sigDetectionDone = newSignal<DetectionDoneSignalType>(s_DETECTION_DONE_SIG);

    newSlot(s_DETECT_MARKER_SLOT, &SArucoTracker::detectMarker, this);
}

//-----------------------------------------------------------------------------

SArucoTracker::~SArucoTracker() throw ()
{
}

//-----------------------------------------------------------------------------

void SArucoTracker::configuring() throw (::fwTools::Failed)
{
    ::fwRuntime::ConfigurationElement::sptr cfg = m_configuration->findConfigurationElement("config");
    SLM_ASSERT("Tag 'config' not found.", cfg);


    // gets marker informations
    {
        ::fwRuntime::ConfigurationElement::sptr cfgTrack = cfg->findConfigurationElement("track");
        SLM_ASSERT("Tag 'track' not found.", cfgTrack);
        typedef ::fwRuntime::ConfigurationElementContainer::Container CfgContainer;
        for(const CfgContainer::value_type& elt : cfgTrack->getElements())
        {
            SLM_ASSERT("Missing 'id' attribute.", elt->hasAttribute("id"));
            const std::string markersIDStr = elt->getAttributeValue("id");
            ::boost::tokenizer<> tok(markersIDStr);
            MarkerIDType markersID;
            for( ::boost::tokenizer<>::iterator it = tok.begin(); it!=tok.end(); ++it)
            {
                const int id = ::boost::lexical_cast< int >(*it);
                markersID.push_back(id);
            }
            m_markers.push_back(markersID);
        }
    }

    // get the debug markers flag
    ::fwRuntime::ConfigurationElement::sptr cfgDebugMarkers = cfg->findConfigurationElement("debugMarkers");
    if (cfgDebugMarkers && cfgDebugMarkers->getValue() == "yes")
    {
        m_debugMarkers = true;
    }

    m_dictionary  = ::cv::aruco::getPredefinedDictionary(::cv::aruco::DICT_ARUCO_ORIGINAL);


}

//-----------------------------------------------------------------------------

void SArucoTracker::starting() throw (::fwTools::Failed)
{
    // initialized marker timeline matrix (4*2D points)
    const size_t numTagTL = this->getKeyGroupSize(s_TAGTL_INOUT_GROUP);
    for(size_t i = 0; i < numTagTL; ++i)
    {
        ::arData::MarkerTL::sptr markerTL = this->getInOut< ::arData::MarkerTL >(s_TAGTL_INOUT_GROUP, i);
        markerTL->initPoolSize(static_cast<unsigned int>(m_markers[i].size()));
    }
}

//-----------------------------------------------------------------------------

void SArucoTracker::stopping() throw (::fwTools::Failed)
{
    m_isInitialized = false;
}

//-----------------------------------------------------------------------------

void SArucoTracker::updating() throw (::fwTools::Failed)
{
}

//-----------------------------------------------------------------------------

void SArucoTracker::detectMarker(::fwCore::HiResClock::HiResClockType timestamp)
{

    // OpenCv Aruco Detection
    std::vector< std::vector< ::cv::Point2f> > markerCorners, rejectedCandidates;

    if (timestamp > m_lastTimestamp)
    {
        ::arData::FrameTL::csptr frameTL = this->getInput< ::arData::FrameTL >(s_FRAMETL_INPUT);
        if(!m_isInitialized)
        {
            ::arData::Camera::csptr arCam = this->getInput< ::arData::Camera >(s_CAMERA_INPUT);
            ::cv::Mat cameraMatrix        = ::cv::Mat::eye(3, 3, CV_64F);
            ::cv::Mat distorsionCoeff     = ::cv::Mat::eye(4, 1, CV_64F);

            // 3x3 matrix (fx 0 cx, 0 fy cy, 0 0 1)
            cameraMatrix.at<double>(0,0) = arCam->getFx();
            cameraMatrix.at<double>(1,1) = arCam->getFy();
            cameraMatrix.at<double>(0,2) = arCam->getCx();
            cameraMatrix.at<double>(1,2) = arCam->getCy();

            //4x1 matrix (k1,k2,p1,p2)
            for (unsigned int i = 0; i < 4; ++i)
            {
                distorsionCoeff.at<double>(static_cast<int>(i),0) = arCam->getDistortionCoefficient()[i];
            }
        }


        ::fwCore::HiResClock::HiResClockType timestamp    = frameTL->getNewerTimestamp();
        const CSPTR(::arData::FrameTL::BufferType) buffer = frameTL->getClosestBuffer(timestamp);

        OSLM_WARN_IF("Buffer not found with timestamp "<< timestamp, !buffer );
        if(buffer)
        {
            m_lastTimestamp = timestamp;

            const std::uint8_t* frameBuff = &buffer->getElement(0);
            std::vector< int > detectedMarkers;

            // read the input image
            const ::cv::Size frameSize(static_cast<int>(frameTL->getWidth()),
                                       static_cast<int>(frameTL->getHeight()));
            ::cv::Mat inImage = ::cv::Mat (frameSize, CV_8UC4, (void*)frameBuff, ::cv::Mat::AUTO_STEP);

            // aruco expects a grey image and make a conversion at the beginning of the detect() method if it receives
            // a RGB image. However we have a RGBA image so we must make the conversion ourselves.
            cv::Mat rgb;
            //inImage is BGRA (see constructor of ::cv::Mat)
            cv::cvtColor(inImage, rgb, CV_BGRA2RGB);

            //Ok, let's detect
            try
            {
                ::cv::aruco::detectMarkers(rgb, m_dictionary, markerCorners, detectedMarkers,m_parameters, rejectedCandidates);
            }
            catch( ::cv::Exception& e )
            {
                const char* err_msg = e.what();
                OSLM_ERROR("Detect marker exception : " << err_msg);
            }

            //for each marker, draw info and its boundaries in the image
            size_t tagTLIndex  = 0;
            for(const auto& markersID : m_markers)
            {
                ::arData::MarkerTL::sptr markerTL =
                    this->getInOut< ::arData::MarkerTL >(s_TAGTL_INOUT_GROUP, tagTLIndex);
                SPTR(::arData::MarkerTL::BufferType) trackerObject;

                unsigned int markerPosition = 0;
                for (const auto& markerID : markersID)
                {
                    for (unsigned int i = 0; i < detectedMarkers.size(); i++)
                    {
                        if (detectedMarkers[i] == markerID)
                        {
                            //Push matrix
                            float markerBuffer[8];
                            for (size_t j = 0; j < 4; ++j)
                            {
                                markerBuffer[j*2]     = markerCorners[i][j].x;
                                markerBuffer[j*2 + 1] = markerCorners[i][j].y;
                            }
                            if(trackerObject == nullptr)
                            {
                                trackerObject = markerTL->createBuffer(timestamp);
                                markerTL->pushObject(trackerObject);
                            }
                            trackerObject->setElement(markerBuffer, markerPosition);
                        }
                    }
                    ++markerPosition;
                }

                //Notify
                if(trackerObject != nullptr)
                {
                    ::arData::TimeLine::ObjectPushedSignalType::sptr sig;
                    sig = markerTL->signal< ::arData::TimeLine::ObjectPushedSignalType >(
                        ::arData::TimeLine::s_OBJECT_PUSHED_SIG );
                    sig->asyncEmit(timestamp);
                }

                ++tagTLIndex;
            }

            //Debugging
            if(m_debugMarkers)
            {
                ::cv::Mat copyIm;
                rgb.copyTo(copyIm);
                try
                {
                    ::cv::aruco::drawDetectedMarkers(copyIm,markerCorners,detectedMarkers);

                    ::cv::namedWindow("Aruco Detection", ::cv::WINDOW_AUTOSIZE);// Create a window for display.
                     ::cv::imshow("Aruco Tag ",copyIm);
                #ifdef WIN32
                    ::cv::waitKey(100);
                #endif

                }
                catch( ::cv::Exception& e )
                {
                    const char* err_msg = e.what();
                    OSLM_ERROR("draw marker exception : " << err_msg);
                }
            }

            //Emit
            m_sigDetectionDone->asyncEmit(timestamp);
        }
    }
}

//-----------------------------------------------------------------------------

::fwServices::IService::KeyConnectionsMap SArucoTracker::getAutoConnections() const
{
    KeyConnectionsMap connections;
    connections.push( s_FRAMETL_INPUT, ::arData::TimeLine::s_OBJECT_PUSHED_SIG, s_DETECT_MARKER_SLOT );

    return connections;
}

} // namespace trackerAruco

