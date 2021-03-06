/* ***** BEGIN LICENSE BLOCK *****
 * FW4SPL - Copyright (C) IRCAD, 2014-2016.
 * Distributed under the terms of the GNU Lesser General Public License (LGPL) as
 * published by the Free Software Foundation.
 * ****** END LICENSE BLOCK ****** */

#ifndef __ARSERVICES_IGRABBER_HPP__
#define __ARSERVICES_IGRABBER_HPP__

#include "arServices/config.hpp"

#include <fwServices/IService.hpp>

namespace arServices
{
/**
 * @brief   This interface defines grabber service API.
 *          Must be implemented for services that grabs frames.
 *
 * @section Signals Signals
 * - \b positionModified(std::int64_t) : Emitted when the position in the video is modified during playing.
 * - \b durationModified(std::int64_t) : Emitted when the duration of the video is modified.
 * - \b cameraStarted() : Emitted when camera is started.
 * - \b cameraStopped() : Emitted when camera is stopped.
 * - \b framePresented() : Emitted when a frame is presented.
 *
 * @section Slots Slots
 * - \b startCamera() : Start playing the camera or the video.
 * - \b stopCamera() : Stop playing the camera or the video.
 * - \b pauseCamera() : Pause the video, it has no effect when playing a camera.
 * - \b loopVideo() : Toggle the loop of the playing.
 * - \b setPositionVideo(int) : Force the current time in the video.
 */
class ARSERVICES_CLASS_API IGrabber : public fwServices::IService
{

public:

    fwCoreServiceClassDefinitionsMacro((IGrabber)(fwServices::IService));

    /**
     * @name Slots API
     * @{
     */
    ARSERVICES_API static const ::fwCom::Slots::SlotKeyType s_START_CAMERA_SLOT;
    ARSERVICES_API static const ::fwCom::Slots::SlotKeyType s_STOP_CAMERA_SLOT;
    ARSERVICES_API static const ::fwCom::Slots::SlotKeyType s_PAUSE_CAMERA_SLOT;
    ARSERVICES_API static const ::fwCom::Slots::SlotKeyType s_SELECT_CAMERA_SLOT;
    ARSERVICES_API static const ::fwCom::Slots::SlotKeyType s_LOOP_VIDEO_SLOT;
    ARSERVICES_API static const ::fwCom::Slots::SlotKeyType s_SET_POSITION_VIDEO_SLOT;
    ARSERVICES_API static const ::fwCom::Slots::SlotKeyType s_PRESENT_SLOT;
    ///@}

    /**
     * @name Signals API
     * @{
     */

    ARSERVICES_API static const ::fwCom::Signals::SignalKeyType s_POSITION_MODIFIED_SIG;
    typedef ::fwCom::Signal<void (int64_t)> PositionModifiedSignalType;

    ARSERVICES_API static const ::fwCom::Signals::SignalKeyType s_DURATION_MODIFIED_SIG;
    typedef ::fwCom::Signal<void (int64_t)> DurationModifiedSignalType;

    ARSERVICES_API static const ::fwCom::Signals::SignalKeyType s_FRAME_PRESENTED_SIG;
    typedef ::fwCom::Signal<void ()> FramePresentedSignalType;

    ARSERVICES_API static const ::fwCom::Signals::SignalKeyType s_CAMERA_STARTED_SIG;
    typedef ::fwCom::Signal<void ()> CameraStartedSignalType;

    ARSERVICES_API static const ::fwCom::Signals::SignalKeyType s_CAMERA_STOPPED_SIG;
    typedef ::fwCom::Signal<void ()> CameraStoppedSignalType;

    /** @} */

    /**
     * @brief Constructor.
     */
    ARSERVICES_API IGrabber() throw ();

    /**
     * @brief Destructor.
     */
    ARSERVICES_API virtual ~IGrabber() throw ();

protected:
    /**
     * @brief API for starting a camera. Needs to be reimplemented in child classes.
     */
    ARSERVICES_API virtual void startCamera() = 0;

    /**
     * @brief API for stopping a camera. Needs to be reimplemented in child classes.
     */
    ARSERVICES_API virtual void stopCamera() = 0;

    /**
     * @brief API for pausing a camera. Needs to be reimplemented in child classes.
     */
    ARSERVICES_API virtual void pauseCamera() = 0;

    /**
     * @brief API for enable/disable the loop mode in video. Needs to be reimplemented in child classes.
     */
    ARSERVICES_API virtual void toggleLoopMode() = 0;

    /**
     * @brief API for setting a new position in the video. Needs to be reimplemented in child classes
     */
    ARSERVICES_API virtual void setPosition(int64_t position) = 0;

};

} //namespace arServices
#endif  // __ARSERVICES_IGRABBER_HPP__
