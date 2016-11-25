/* ***** BEGIN LICENSE BLOCK *****
 * FW4SPL - Copyright (C) IRCAD, 2014-2016.
 * Distributed under the terms of the GNU Lesser General Public License (LGPL) as
 * published by the Free Software Foundation.
 * ****** END LICENSE BLOCK ****** */

#ifndef __TRACKERARUCO_SARUCOTRACKER_HPP__
#define __TRACKERARUCO_SARUCOTRACKER_HPP__

#include "trackerAruco/config.hpp"

#include <tracker/ITracker.hpp>

#include <fwCom/Slot.hpp>
#include <fwCom/Slots.hpp>
#include <fwData/Composite.hpp>

#include <fwCore/HiResClock.hpp>

#include <fwServices/macros.hpp>

#include <opencv2/aruco.hpp>

namespace trackerAruco
{

/**
 * @brief   Class used to track multiple tags with ArUco.
 *
 * @section Signals Signals
 * - \b detectionDone(::fwCore::HiResClock::HiResClockType) : This signal is emitted when the tracker find tags.
 *
 * @section XML XML Configuration
 *
 * @code{.xml}
        <service uid="..." type="::trackerAruco::SArucoTracker" >
            <in key="frameTL" uid="frameTLUid" autoConnect="yes"/>
            <in key="camera" uid="cameraUid" />
            <inout group="tagTL" >
                <key uid="WireTimeline" />
                <key uid="PatientTimeline" />
                <key uid="TableTimeline" />
            </inout>
            <config>
                <track>
                    <markers id="42,1,100,54" />
                    <markers id="32,10" />
                    <markers id="52,45" />
                </track>
                <threshold>
                    <method>ADPT_THRES</method>
                    <blockSize>7</blockSize>
                    <constant>7</constant>
                </threshold>
                <patternWidth>106</patternWidth>
                <debugMarkers>yes</debugMarkers>
            </config>
        </service>
   @endcode
 * @subsection Input Input
 * - \b frameTL [::arData::FrameTL]: camera used to display video.
 * - \b camera [::arData::Camera]: camera calibration.
 *
 * @subsection In-Out In-Out
 * - \b tagTL [::arData::MarkerTL]: list of markers timelines where to extract the tags. The number of tagTL inout keys
 * must match the number of \b markers entries in the config below.
 *
 * @subsection Configuration Configuration
 *  - \b track (mandatory)
 *      - \b markers (mandatory) : list of the tracked markers.
 *           - \b id (mandatory) : ids of the markers to detect.
 *  - \b threshold (optional)
 *      - \b method (mandatory): can be ADPT_THRES, FIXED_THRES or CANNY.
 *      - \b blockSize : parameter of the chosen method
 *      - \b constant : parameter of the chosen method
 *  - \b patternWidth (optional): width of the pattern(s).
 *  - \b debugMarkers : if value is yes markers debugging mode is activated.
 */
class TRACKERARUCO_CLASS_API SArucoTracker : public ::tracker::ITracker
{
public:

    fwCoreServiceClassDefinitionsMacro((SArucoTracker)(tracker::ITracker));

    typedef ::fwCom::Signal< void (::fwCore::HiResClock::HiResClockType timestamp) > DetectionDoneSignalType;

    /// Key in m_signals map of signal m_sigDetectionDone
    TRACKERARUCO_API static const ::fwCom::Signals::SignalKeyType s_DETECTION_DONE_SIG;

    ///Slot called when debug mode is enabled/disabled
    TRACKERARUCO_API static const ::fwCom::Slots::SlotKeyType s_DISPLAY_TAGS_SLOT;

    TRACKERARUCO_API static const ::fwCom::Slots::SlotKeyType s_DETECT_MARKER_SLOT;

    typedef std::vector< int >          MarkerIDType;
    typedef std::vector< MarkerIDType > MarkerIDVectorType;

    /**
     * @brief Constructor.
     */
    TRACKERARUCO_API SArucoTracker() throw ();

    /**
     * @brief Destructor.
     */
    TRACKERARUCO_API virtual ~SArucoTracker() throw ();

    /**
     * @brief Returns proposals to connect service slots to associated object signals,
     * this method is used for obj/srv auto connection
     *
     * Connect TimeLine::s_OBJECT_PUSHED_SIG to this::s_DETECT_MARKER_SLOT
     */
    TRACKERARUCO_API virtual KeyConnectionsMap getAutoConnections() const;

protected:
    /**
     * @brief Configuring method : This method is used to configure the service.
     */
    TRACKERARUCO_API void configuring() throw (fwTools::Failed);

    /**
     * @brief Starting method : This method is used to initialize the service.
     */
    TRACKERARUCO_API void starting() throw (fwTools::Failed);

    /**
     * @brief Updating method : This method is used to update the service.
     */
    TRACKERARUCO_API void updating() throw (fwTools::Failed);

    /**
     * @brief Stopping method : This method is used to stop the service.
     */
    TRACKERARUCO_API void stopping() throw (fwTools::Failed);

    /// Detect marker slot
    void detectMarker(::fwCore::HiResClock::HiResClockType timestamp);
private:


    /// Last timestamp
    ::fwCore::HiResClock::HiResClockType m_lastTimestamp;

    /// True if tracker is initialized
    bool m_isInitialized;

    bool m_debugMarkers;

    ::cv::aruco::DetectorParameters  m_parameters;
    ::cv::aruco::Dictionary  m_dictionary;

    /// Marker vector [[0,1,2],[4,5,6]]
    MarkerIDVectorType m_markers;
    /// Signal to emit when
    DetectionDoneSignalType::sptr m_sigDetectionDone;
};

} //namespace trackerAruco

#endif /* __TRACKERARUCO_SARUCOTRACKER_HPP__ */
