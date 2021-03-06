/* ***** BEGIN LICENSE BLOCK *****
 * FW4SPL - Copyright (C) IRCAD, 2016.
 * Distributed under the terms of the GNU Lesser General Public License (LGPL) as
 * published by the Free Software Foundation.
 * ****** END LICENSE BLOCK ****** */

#ifndef __VALIDATORS_UT_MONOCAMERATEST_HPP__
#define __VALIDATORS_UT_MONOCAMERATEST_HPP__

#include <cppunit/extensions/HelperMacros.h>

namespace validators
{
namespace ut
{

/// Test the MonoCamera validator
class MonoCameraTest : public CPPUNIT_NS::TestFixture
{
CPPUNIT_TEST_SUITE( MonoCameraTest );
CPPUNIT_TEST( testValidator );
CPPUNIT_TEST_SUITE_END();

public:
    // interface
    void setUp();
    void tearDown();

    void testValidator();

};

} //namespace ut
} //namespace validators

#endif //__VALIDATORS_UT_MONOCAMERATEST_HPP__

