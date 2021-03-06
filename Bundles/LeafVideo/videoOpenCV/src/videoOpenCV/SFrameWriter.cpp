/* ***** BEGIN LICENSE BLOCK *****
 * FW4SPL - Copyright (C) IRCAD, 2016.
 * Distributed under the terms of the GNU Lesser General Public License (LGPL) as
 * published by the Free Software Foundation.
 * ****** END LICENSE BLOCK ****** */

#include "videoOpenCV/SFrameWriter.hpp"

#include <fwCom/Slot.hpp>
#include <fwCom/Slot.hxx>
#include <fwCom/Slots.hpp>
#include <fwCom/Slots.hxx>

#include <fwData/Composite.hpp>
#include <fwData/location/Folder.hpp>

#include <fwServices/macros.hpp>

#include <fwGui/dialog/LocationDialog.hpp>
#include <fwGui/dialog/MessageDialog.hpp>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <boost/filesystem/operations.hpp>

namespace videoOpenCV
{

fwServicesRegisterMacro( ::io::IWriter, ::videoOpenCV::SFrameWriter, ::arData::FrameTL);

static const ::fwCom::Slots::SlotKeyType s_SAVE_FRAME   = "saveFrame";
static const ::fwCom::Slots::SlotKeyType s_START_RECORD = "startRecord";
static const ::fwCom::Slots::SlotKeyType s_STOP_RECORD  = "stopRecord";

//------------------------------------------------------------------------------

SFrameWriter::SFrameWriter() throw() :
    m_imageType(0),
    m_isRecording(false)
{
    newSlot(s_SAVE_FRAME, &SFrameWriter::saveFrame, this);
    newSlot(s_START_RECORD, &SFrameWriter::startRecord, this);
    newSlot(s_STOP_RECORD, &SFrameWriter::stopRecord, this);
}

//------------------------------------------------------------------------------

SFrameWriter::~SFrameWriter() throw()
{
}

//------------------------------------------------------------------------------

::io::IOPathType SFrameWriter::getIOPathType() const
{
    return ::io::FOLDER;
}

//------------------------------------------------------------------------------

void SFrameWriter::configuring() throw(::fwTools::Failed)
{
}

//------------------------------------------------------------------------------

void SFrameWriter::starting() throw(::fwTools::Failed)
{
}

//------------------------------------------------------------------------------

void SFrameWriter::configureWithIHM()
{
    static ::boost::filesystem::path _sDefaultPath("");
    ::fwGui::dialog::LocationDialog dialogFile;
    dialogFile.setTitle("Choose a folder to save the frames");
    dialogFile.setDefaultLocation( ::fwData::location::Folder::New(_sDefaultPath) );
    dialogFile.setOption(::fwGui::dialog::ILocationDialog::WRITE);
    dialogFile.setType(::fwGui::dialog::ILocationDialog::FOLDER);

    ::fwData::location::Folder::sptr result;
    result = ::fwData::location::Folder::dynamicCast( dialogFile.show() );
    if (result)
    {
        _sDefaultPath = result->getFolder().parent_path();
        dialogFile.saveDefaultLocation( ::fwData::location::Folder::New(_sDefaultPath) );
        this->setFolder(result->getFolder());
    }
    else
    {
        this->clearLocations();
    }

}

//------------------------------------------------------------------------------

void SFrameWriter::stopping() throw(::fwTools::Failed)
{
    this->stopRecord();
}

//------------------------------------------------------------------------------

void SFrameWriter::updating() throw(::fwTools::Failed)
{

    ::fwCore::HiResClock::HiResClockType timestamp = ::fwCore::HiResClock::getTimeInMilliSec();
    this->startRecord();
    this->saveFrame(timestamp);
    this->stopRecord();
}

//------------------------------------------------------------------------------

void SFrameWriter::saveFrame(::fwCore::HiResClock::HiResClockType timestamp)
{
    if (m_isRecording)
    {
        ::arData::FrameTL::csptr frameTL = this->getInput< ::arData::FrameTL >(::io::s_DATA_KEY);

        // Get the buffer of the copied timeline
        CSPTR(::arData::FrameTL::BufferType) buffer = frameTL->getClosestBuffer(timestamp);

        if (buffer)
        {
            int width  = static_cast<int>( frameTL->getWidth() );
            int height = static_cast<int>( frameTL->getHeight() );

            const ::boost::uint8_t* imageBuffer = &buffer->getElement(0);

            ::cv::Mat image(::cv::Size(width, height), m_imageType, (void*)imageBuffer, ::cv::Mat::AUTO_STEP);

            size_t time = static_cast<size_t>(timestamp);
            std::string filename( "img_" + std::to_string(time) + ".tiff");
            ::boost::filesystem::path path = this->getFolder() / filename;

            if (image.type() == CV_8UC3)
            {
                // convert the read image from BGR to RGB
                ::cv::Mat imageRgb;
                ::cv::cvtColor(image, imageRgb, ::cv::COLOR_BGR2RGB);
                ::cv::imwrite(path.string(), imageRgb);
            }
            else if (image.type() == CV_8UC4)
            {
                // convert the read image from BGRA to RGBA
                ::cv::Mat imageRgb;
                ::cv::cvtColor(image, imageRgb, ::cv::COLOR_BGRA2RGBA);
                ::cv::imwrite(path.string(), imageRgb);
            }
            else
            {
                ::cv::imwrite(path.string(), image);
            }
        }
    }
}

//------------------------------------------------------------------------------

void SFrameWriter::startRecord()
{
    if (!this->hasLocationDefined())
    {
        this->configureWithIHM();
    }

    if (this->hasLocationDefined())
    {
        ::arData::FrameTL::csptr frameTL = this->getInput< ::arData::FrameTL >(::io::s_DATA_KEY);

        if (frameTL->getType() == ::fwTools::Type::s_UINT8 && frameTL->getNumberOfComponents() == 3)
        {
            m_imageType = CV_8UC3;
        }
        if (frameTL->getType() == ::fwTools::Type::s_UINT8 && frameTL->getNumberOfComponents() == 4)
        {
            m_imageType = CV_8UC4;
        }
        else if (frameTL->getType() == ::fwTools::Type::s_UINT8 && frameTL->getNumberOfComponents() == 1)
        {
            m_imageType = CV_8UC1;
        }
        else if (frameTL->getType() == ::fwTools::Type::s_UINT16 && frameTL->getNumberOfComponents() == 1)
        {
            m_imageType = CV_16UC1;
        }
        else
        {
            OSLM_ERROR("This type of frame : " + frameTL->getType().string() + " with " +
                       std::to_string(frameTL->getNumberOfComponents()) + " is not supported");
            return;
        }

        ::boost::filesystem::path path = this->getFolder();

        if (!::boost::filesystem::exists(path))
        {
            ::boost::filesystem::create_directories(path);
        }

        m_isRecording = true;
    }
}

//------------------------------------------------------------------------------

void SFrameWriter::stopRecord()
{
    m_isRecording = false;
}

//------------------------------------------------------------------------------

::fwServices::IService::KeyConnectionsMap SFrameWriter::getAutoConnections() const
{
    ::fwServices::IService::KeyConnectionsMap connections;
    connections.push(::io::s_DATA_KEY, ::arData::FrameTL::s_OBJECT_PUSHED_SIG, s_SAVE_FRAME);
    return connections;
}

//------------------------------------------------------------------------------

} // namespace videoOpenCV
