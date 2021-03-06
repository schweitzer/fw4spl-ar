/* ***** BEGIN LICENSE BLOCK *****
 * FW4SPL - Copyright (C) IRCAD, 2014-2016.
 * Distributed under the terms of the GNU Lesser General Public License (LGPL) as
 * published by the Free Software Foundation.
 * ****** END LICENSE BLOCK ****** */

#include "videoCalibration/SOpenCVExtrinsic.hpp"

#include <arData/CalibrationInfo.hpp>
#include <arData/Camera.hpp>
#include <arData/CameraSeries.hpp>

#include <fwCom/Slot.hxx>
#include <fwCom/Slots.hxx>

#include <fwData/PointList.hpp>
#include <fwData/TransformationMatrix3D.hpp>
#include <fwData/mt/ObjectReadLock.hpp>
#include <fwData/mt/ObjectWriteLock.hpp>

#include <fwPreferences/helper.hpp>

#include <fwRuntime/ConfigurationElement.hpp>

#include <fwServices/IService.hpp>
#include <fwServices/macros.hpp>

#include <fwTools/Object.hpp>
#include <fwTools/fwID.hpp>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

fwServicesRegisterMacro(::videoCalibration::ICalibration, ::videoCalibration::SOpenCVExtrinsic, ::arData::CameraSeries);

namespace videoCalibration
{

static const ::fwCom::Slots::SlotKeyType s_UPDATE_CHESSBOARD_SIZE_SLOT = "updateChessboardSize";

// ----------------------------------------------------------------------------

SOpenCVExtrinsic::SOpenCVExtrinsic() throw () : m_width(11),
                                                m_height(8),
                                                m_squareSize(20.0),
                                                m_camIndex(1)
{
    newSlot(s_UPDATE_CHESSBOARD_SIZE_SLOT, &SOpenCVExtrinsic::updateChessboardSize, this);
}

// ----------------------------------------------------------------------------

SOpenCVExtrinsic::~SOpenCVExtrinsic() throw ()
{
}

//------------------------------------------------------------------------------

void SOpenCVExtrinsic::configuring() throw (fwTools::Failed)
{
    ::fwRuntime::ConfigurationElement::sptr cfgIdx = m_configuration->findConfigurationElement("camIndex");
    if(cfgIdx)
    {
        std::string idxStr = cfgIdx->getValue();
        SLM_ASSERT("'camIndex' is empty.", !idxStr.empty());
        m_camIndex = ::boost::lexical_cast<size_t>(idxStr);
    }

    ::fwRuntime::ConfigurationElement::sptr cfgBoard = m_configuration->findConfigurationElement("board");
    SLM_ASSERT("Tag 'board' not found.", cfgBoard);

    SLM_ASSERT("Attribute 'width' is missing.", cfgBoard->hasAttribute("width"));
    m_widthKey = cfgBoard->getAttributeValue("width");
    SLM_ASSERT("Attribute 'width' is empty", !m_widthKey.empty());

    SLM_ASSERT("Attribute 'height' is missing.", cfgBoard->hasAttribute("height"));
    m_heightKey = cfgBoard->getAttributeValue("height");
    SLM_ASSERT("Attribute 'height' is empty", !m_heightKey.empty());

    if( cfgBoard->hasAttribute("squareSize"))
    {
        m_squareSizeKey = cfgBoard->getAttributeValue("squareSize");
        SLM_ASSERT("Attribute 'squareSize' is empty", !m_squareSizeKey.empty());
    }
}

// ----------------------------------------------------------------------------

void SOpenCVExtrinsic::starting() throw (fwTools::Failed)
{
    this->updateChessboardSize();
}

// ----------------------------------------------------------------------------

void SOpenCVExtrinsic::stopping() throw (fwTools::Failed)
{
}

//------------------------------------------------------------------------------

void SOpenCVExtrinsic::swapping() throw (fwTools::Failed)
{
    this->stopping();
    this->starting();
}

//--------------------------------------------------------------------- ---------

void SOpenCVExtrinsic::updating() throw (fwTools::Failed)
{
    ::arData::CameraSeries::sptr camSeries = this->getInOut< ::arData::CameraSeries >("cameraSeries");

    SLM_ASSERT("camera index must be > 0 and < camSeries->getNumberOfCameras()",
               m_camIndex > 0 && m_camIndex < camSeries->getNumberOfCameras());

    ::arData::CalibrationInfo::csptr calInfo1 = this->getInput< ::arData::CalibrationInfo>("calibrationInfo1");
    ::arData::CalibrationInfo::csptr calInfo2 = this->getInput< ::arData::CalibrationInfo>("calibrationInfo2");

    SLM_ASSERT("Object with 'calibrationInfo1' is not found", calInfo1);
    SLM_ASSERT("Object with 'calibrationInfo2' is not found", calInfo2);

    SLM_WARN_IF("Calibration info is empty.", calInfo1->getPointListContainer().empty());
    if(!calInfo1->getPointListContainer().empty())
    {
        std::vector<std::vector< ::cv::Point3f > > objectPoints;

        std::vector< ::cv::Point3f > points;
        for (unsigned int y = 0; y < m_height - 1; ++y)
        {
            for (unsigned int x = 0; x < m_width - 1; ++x)
            {
                points.push_back(::cv::Point3f(float(y*m_squareSize), float(x*m_squareSize), 0));
            }
        }

        std::vector<std::vector< ::cv::Point2f > > imagePoints1;
        std::vector<std::vector< ::cv::Point2f > > imagePoints2;
        {
            ::fwData::mt::ObjectReadLock calInfo1Lock(calInfo1);
            ::fwData::mt::ObjectReadLock calInfo2Lock(calInfo2);

            ::arData::CalibrationInfo::PointListContainerType ptlists1 = calInfo1->getPointListContainer();
            ::arData::CalibrationInfo::PointListContainerType ptlists2 = calInfo2->getPointListContainer();

            SLM_ASSERT("The two calibrationInfo have not the same size", ptlists1.size() == ptlists2.size());

            ::arData::CalibrationInfo::PointListContainerType::iterator itr1   = ptlists1.begin();
            ::arData::CalibrationInfo::PointListContainerType::iterator itr2   = ptlists2.begin();
            ::arData::CalibrationInfo::PointListContainerType::iterator itrEnd = ptlists1.end();


            for(; itr1 != itrEnd; ++itr1, ++itr2)
            {
                ::fwData::PointList::sptr ptList1 = *itr1;
                ::fwData::PointList::sptr ptList2 = *itr2;
                std::vector< ::cv::Point2f > imgPoint1;
                std::vector< ::cv::Point2f > imgPoint2;

                for(fwData::Point::csptr point : ptList1->getCRefPoints())
                {
                    SLM_ASSERT("point is null", point);
                    imgPoint1.push_back(::cv::Point2f(
                                            static_cast<float>(point->getCRefCoord()[0]),
                                            static_cast<float>(point->getCRefCoord()[1])));
                }

                for(fwData::Point::csptr point : ptList2->getCRefPoints())
                {
                    SLM_ASSERT("point is null", point);
                    imgPoint2.push_back(::cv::Point2f(
                                            static_cast<float>(point->getCRefCoord()[0]),
                                            static_cast<float>(point->getCRefCoord()[1])));
                }

                imagePoints1.push_back(imgPoint1);
                imagePoints2.push_back(imgPoint2);
                objectPoints.push_back(points);
            }
        }

        // Set the cameras
        ::cv::Mat cameraMatrix1 = ::cv::Mat::eye(3, 3, CV_64F);
        ::cv::Mat cameraMatrix2 = ::cv::Mat::eye(3, 3, CV_64F);

        std::vector<float> distortionCoefficients1(5);
        std::vector<float> distortionCoefficients2(5);
        ::cv::Mat rotationMatrix;
        ::cv::Mat translationVector;
        ::cv::Mat essentialMatrix;
        ::cv::Mat fundamentalMatrix;

        ::fwData::Image::sptr img = calInfo1->getImageContainer().front();
        ::cv::Size2i imgsize(static_cast<int>(img->getSize()[0]), static_cast<int>(img->getSize()[1]));
        {

            ::fwData::mt::ObjectReadLock camSeriesLock(camSeries);
            ::arData::Camera::sptr cam1 = camSeries->getCamera(0);
            ::arData::Camera::sptr cam2 = camSeries->getCamera(m_camIndex);

            ::fwData::mt::ObjectReadLock cam1Lock(cam1);
            ::fwData::mt::ObjectReadLock cam2Lock(cam2);

            cameraMatrix1.at<double>(0,0) = cam1->getFx();
            cameraMatrix1.at<double>(1,1) = cam1->getFy();
            cameraMatrix1.at<double>(0,2) = cam1->getCx();
            cameraMatrix1.at<double>(1,2) = cam1->getCy();

            cameraMatrix2.at<double>(0,0) = cam2->getFx();
            cameraMatrix2.at<double>(1,1) = cam2->getFy();
            cameraMatrix2.at<double>(0,2) = cam2->getCx();
            cameraMatrix2.at<double>(1,2) = cam2->getCy();
            for (size_t i = 0; i < 5; ++i)
            {
                distortionCoefficients1[i] = static_cast<float>(cam1->getDistortionCoefficient()[i]);
                distortionCoefficients2[i] = static_cast<float>(cam2->getDistortionCoefficient()[i]);
            }
        }
        double err = ::cv::stereoCalibrate(objectPoints, imagePoints1, imagePoints2,
                                           cameraMatrix1, distortionCoefficients1,
                                           cameraMatrix2, distortionCoefficients2,
                                           imgsize, rotationMatrix, translationVector, essentialMatrix,
                                           fundamentalMatrix,
                                           CV_CALIB_FIX_INTRINSIC,
                                           ::cv::TermCriteria(::cv::TermCriteria::MAX_ITER + ::cv::TermCriteria::EPS,
                                                              100, 1e-5));

        OSLM_DEBUG("Calibration error :" << err);

        ::fwData::TransformationMatrix3D::sptr matrix = ::fwData::TransformationMatrix3D::New();
        for (size_t i = 0; i<3; ++i)
        {
            for (size_t j = 0; j<3; ++j)
            {
                matrix->setCoefficient(i,j, rotationMatrix.at<double>(static_cast<int>(i),static_cast<int>(j)));
            }
        }
        matrix->setCoefficient(0,3, translationVector.at<double>(0,0));
        matrix->setCoefficient(1,3, translationVector.at<double>(1,0));
        matrix->setCoefficient(2,3, translationVector.at<double>(2,0));

        {
            ::fwData::mt::ObjectWriteLock camSeriesLock(camSeries);
            camSeries->setExtrinsicMatrix(m_camIndex, matrix);
        }

        ::arData::CameraSeries::ExtrinsicCalibratedSignalType::sptr sig;
        sig = camSeries->signal< ::arData::CameraSeries::ExtrinsicCalibratedSignalType > (
            ::arData::CameraSeries::s_EXTRINSIC_CALIBRATED_SIG);

        sig->asyncEmit();
    }
}

//------------------------------------------------------------------------------

void SOpenCVExtrinsic::updateChessboardSize()
{
    const std::string widthStr = ::fwPreferences::getPreference(m_widthKey);
    if(!widthStr.empty())
    {
        m_width = std::stoi(widthStr);
    }
    const std::string heightStr = ::fwPreferences::getPreference(m_heightKey);
    if(!heightStr.empty())
    {
        m_height = std::stoi(heightStr);
    }
    const std::string squareSizeStr = ::fwPreferences::getPreference(m_squareSizeKey);
    if(!squareSizeStr.empty())
    {
        m_squareSize = std::stof(squareSizeStr);
    }
}

//------------------------------------------------------------------------------

} // namespace videoCalibration
