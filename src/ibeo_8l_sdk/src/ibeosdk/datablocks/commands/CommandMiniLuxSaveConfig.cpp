//======================================================================
/*! \file CommandMiniLuxSaveConfig.cpp
 *
 * \copydoc Copyright
 * \author Jan Christian Dittmer (jcd)
 * \date Apr 10, 2015
 *///-------------------------------------------------------------------
//======================================================================

#include <ibeosdk/datablocks/commands/CommandMiniLuxSaveConfig.hpp>

#include <ibeosdk/io.hpp>

//======================================================================

namespace ibeosdk {

//======================================================================

CommandMiniLuxSaveConfig::CommandMiniLuxSaveConfig()
  : MiniLuxCommand<CommandId::CmdLuxSaveConfig>(),
    m_reserved(0x0000)
{}

//======================================================================

bool CommandMiniLuxSaveConfig::deserialize(std::istream& is, const IbeoDataHeader& dh)
{
	const std::istream::pos_type startPos = is.tellg();

	readLE(is, m_commandId);
	readLE(is, m_reserved);

	return !is.fail()
	       && ((is.tellg() - startPos) == this->getSerializedSize())
	       && this->getSerializedSize() == dh.getMessageSize();
}

//======================================================================

bool CommandMiniLuxSaveConfig::serialize(std::ostream& os) const
{
	const std::istream::pos_type startPos = os.tellp();

	writeLE(os, m_commandId);
	writeLE(os, m_reserved);

	return !os.fail() && ((os.tellp() - startPos) == this->getSerializedSize());
}

//======================================================================

}// namespace ibeosdk

//======================================================================
