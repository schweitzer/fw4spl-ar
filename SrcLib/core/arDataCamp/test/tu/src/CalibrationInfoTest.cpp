/* ***** BEGIN LICENSE BLOCK *****
 * FW4SPL - Copyright (C) IRCAD, 2014-2015.
 * Distributed under the terms of the GNU Lesser General Public License (LGPL) as
 * published by the Free Software Foundation.
 * ****** END LICENSE BLOCK ****** */

#include "DataCampHelper.hpp"
#include "CalibrationInfoTest.hpp"

#include <arData/CalibrationInfo.hpp>

#include <fwData/Image.hpp>
#include <fwData/Point.hpp>

#include <fwTest/generator/Image.hpp>

#include <arDataCamp/Version.hpp>


// Registers the fixture into the 'registry'
CPPUNIT_TEST_SUITE_REGISTRATION( ::arDataCamp::ut::CalibrationInfoTest );

namespace arDataCamp
{
namespace ut
{

void CalibrationInfoTest::setUp()
{
    // Set up context before running a test.
}

void CalibrationInfoTest::tearDown()
{
    //Force link with arDataCamp
    const int arfVersion = ::arDataCamp::Version::s_CURRENT_VERSION;
}

//------------------------------------------------------------------------------

void CalibrationInfoTest::propertiesTest()
{
    ::arData::CalibrationInfo::sptr calInfo = ::arData::CalibrationInfo::New();

    ::fwData::Image::sptr img = ::fwData::Image::New();
    ::fwTest::generator::Image::generateRandomImage( img, ::fwTools::Type::s_INT16);

    ::fwData::PointList::sptr pl = ::fwData::PointList::New();

    ::fwData::Point::sptr pt1 = ::fwData::Point::New( 1.0, 2.0, 3.0 );
    ::fwData::Point::sptr pt2 = ::fwData::Point::New( 4.0, 5.0, 6.0 );
    ::fwData::Point::sptr pt3 = ::fwData::Point::New( 7.0, 8.0, 9.0 );

    pl->getRefPoints().push_back(pt1);
    pl->getRefPoints().push_back(pt2);
    pl->getRefPoints().push_back(pt3);

    calInfo->addRecord(img, pl);


    const DataCampHelper::PropertiesNameType dataProperties = { "fields",
                                                                "image_container",
                                                                "pointlist_container"};

    DataCampHelper::visitProperties(calInfo->getClassname(), dataProperties);
    DataCampHelper::compareObjectPropertyValue(calInfo, "@image_container.0", img);
    DataCampHelper::compareObjectPropertyValue(calInfo, "@pointlist_container.0", pl);
    DataCampHelper::compareObjectPropertyValue(calInfo, "@pointlist_container.0.points.0", pt1);
    DataCampHelper::compareObjectPropertyValue(calInfo, "@pointlist_container.0.points.1", pt2);
    DataCampHelper::compareObjectPropertyValue(calInfo, "@pointlist_container.0.points.2", pt3);

}

//------------------------------------------------------------------------------

} //namespace ut
} //namespace arDataCamp
