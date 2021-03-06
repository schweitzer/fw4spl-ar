/* ***** BEGIN LICENSE BLOCK *****
 * FW4SPL - Copyright (C) IRCAD, 2014-2016.
 * Distributed under the terms of the GNU Lesser General Public License (LGPL) as
 * published by the Free Software Foundation.
 * ****** END LICENSE BLOCK ****** */

#include "arServices/IGrabber.hpp"

#include <fwCom/Signal.hxx>
#include <fwCom/Slots.hxx>

namespace arServices
{

const ::fwCom::Signals::SignalKeyType IGrabber::s_POSITION_MODIFIED_SIG = "positionModified";
const ::fwCom::Signals::SignalKeyType IGrabber::s_DURATION_MODIFIED_SIG = "durationModified";

const ::fwCom::Signals::SignalKeyType IGrabber::s_CAMERA_STARTED_SIG = "cameraStarted";
const ::fwCom::Signals::SignalKeyType IGrabber::s_CAMERA_STOPPED_SIG = "cameraStopped";

const ::fwCom::Signals::SignalKeyType IGrabber::s_FRAME_PRESENTED_SIG = "framePresented";

const ::fwCom::Slots::SlotKeyType IGrabber::s_START_CAMERA_SLOT       = "startCamera";
const ::fwCom::Slots::SlotKeyType IGrabber::s_STOP_CAMERA_SLOT        = "stopCamera";
const ::fwCom::Slots::SlotKeyType IGrabber::s_PAUSE_CAMERA_SLOT       = "pauseCamera";
const ::fwCom::Slots::SlotKeyType IGrabber::s_LOOP_VIDEO_SLOT         = "loopVideo";
const ::fwCom::Slots::SlotKeyType IGrabber::s_SET_POSITION_VIDEO_SLOT = "setPositionVideo";

// ----------------------------------------------------------------------------

IGrabber::IGrabber() throw ()
{
    //Declare all signals
    newSignal< PositionModifiedSignalType >( s_POSITION_MODIFIED_SIG );
    newSignal< DurationModifiedSignalType >( s_DURATION_MODIFIED_SIG );
    newSignal< CameraStartedSignalType >   ( s_CAMERA_STARTED_SIG    );
    newSignal< CameraStoppedSignalType >   ( s_CAMERA_STOPPED_SIG    );
    newSignal< FramePresentedSignalType >  ( s_FRAME_PRESENTED_SIG   );

    newSlot( s_START_CAMERA_SLOT, &IGrabber::startCamera, this );
    newSlot( s_STOP_CAMERA_SLOT, &IGrabber::stopCamera, this );
    newSlot( s_PAUSE_CAMERA_SLOT, &IGrabber::pauseCamera, this );
    newSlot( s_LOOP_VIDEO_SLOT, &IGrabber::toggleLoopMode, this );
    newSlot( s_SET_POSITION_VIDEO_SLOT, &IGrabber::setPosition, this );

}

// ----------------------------------------------------------------------------

IGrabber::~IGrabber() throw ()
{

}

// ----------------------------------------------------------------------------

}  // namespace arServices
