
set( NAME validatorsTest )
set( VERSION  )
set( TYPE TEST )
set( DEPENDENCIES
    fwCore
    fwTest
    fwData
    fwMedData
    fwActivities
    arData
)
set( REQUIREMENTS
    validators
)

set(CPPUNITTEST_OPTIONS BUNDLE validators WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
