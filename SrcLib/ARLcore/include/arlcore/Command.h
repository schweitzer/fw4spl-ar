/* ***** BEGIN LICENSE BLOCK *****
 * FW4SPL - Copyright (C) IRCAD, 2009-2016.
 * Distributed under the terms of the GNU Lesser General Public License (LGPL) as
 * published by the Free Software Foundation.
 * ****** END LICENSE BLOCK ****** */

#ifndef __ARLCORE_COMMAND_H__
#define __ARLCORE_COMMAND_H__
#include <arlcore/Common.h>

#include <vector>
#include <string>
#include <map>

namespace arlCore
{
class Command
{
/**
 * @brief   User interactor
 */
public:
    ARLCORE_API Command( const std::string& fileName = "" );

    ARLCORE_API ~Command( void );

    //! @return The value associated to a command
    ARLCORE_API bool get( const std::string& commandName );

    //! @brief Refresh with key values
    ARLCORE_API bool refresh( void );

protected:
    /**
     * @brief Set to true the command
     * @return false if the commandName doesn't exist
     */
    //ARLCORE_API bool set( const std::string& commandName );

private:
    //! @brief Values
    std::vector<bool> m_values;

    //! @brief Index in m_values by command name
    std::map< std::string, unsigned int > m_byName;

    //! @brief Index in m_values by key
    std::map< char, unsigned int > m_byKey;
};
} // namespace arlCore
#endif // __ARLCORE_COMMAND_H__
