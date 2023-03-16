
#ifndef ARTERY_MCSERVICE_H_
#define ARTERY_MCSERVICE_H_

#include "artery/application/ItsG5BaseService.h"
#include "artery/utility/Channel.h"
#include "artery/utility/Geometry.h"
#include <vanetza/asn1/mcm.hpp>
#include <vanetza/btp/data_interface.hpp>
#include <vanetza/units/angle.hpp>
#include <vanetza/units/velocity.hpp>
#include <omnetpp/simtime.h>

namespace artery
{

class NetworkInterfaceTable;
class Timer;
class VehicleDataProvider;

class McService : public ItsG5BaseService
{
	public:
		McService();
		void initialize() override;
		void indicate(const vanetza::btp::DataIndication&, std::unique_ptr<vanetza::UpPacket>) override;
		void trigger() override;

	private:
		void checkTriggeringConditions(const omnetpp::SimTime&);
		bool checkHeadingDelta() const;
		bool checkPositionDelta() const;
		bool checkSpeedDelta() const;
		void sendMcm(const omnetpp::SimTime&);
		omnetpp::SimTime genMcmDcc();

		ChannelNumber mPrimaryChannel = channel::CCH;
		const NetworkInterfaceTable* mNetworkInterfaceTable = nullptr;
		const VehicleDataProvider* mVehicleDataProvider = nullptr;
		const Timer* mTimer = nullptr;
		LocalDynamicMap* mLocalDynamicMap = nullptr;

		omnetpp::SimTime mGenMcmMin;
		omnetpp::SimTime mGenMcmMax;
		omnetpp::SimTime mGenMcm;
		unsigned mGenMcmLowDynamicsCounter;
		unsigned mGenMcmLowDynamicsLimit;
		Position mLastMcmPosition;
		vanetza::units::Velocity mLastMcmSpeed;
		vanetza::units::Angle mLastMcmHeading;
		omnetpp::SimTime mLastMcmTimestamp;
		omnetpp::SimTime mLastLowMcmTimestamp;
		vanetza::units::Angle mHeadingDelta;
		vanetza::units::Length mPositionDelta;
		vanetza::units::Velocity mSpeedDelta;
		bool mDccRestriction;
		bool mFixedRate;
};

vanetza::asn1::Mcm createManoeuvreCoordinationMessage(const VehicleDataProvider&, uint16_t genDeltaTime);

} // namespace artery

#endif /* ARTERY_MCSERVICE_H_ */