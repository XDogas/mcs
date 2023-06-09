
#include "artery/application/McObject.h"
#include "artery/application/McService.h"
#include "artery/application/Asn1PacketVisitor.h"
#include "artery/application/MultiChannelPolicy.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/utility/simtime_cast.h"
#include "veins/base/utils/Coord.h"
#include <boost/units/cmath.hpp>
#include <boost/units/systems/si/prefixes.hpp>
#include <omnetpp/cexception.h>
#include <vanetza/btp/ports.hpp>
#include <vanetza/dcc/transmission.hpp>
#include <vanetza/dcc/transmit_rate_control.hpp>
#include <vanetza/facilities/cam_functions.hpp> // change
#include <chrono>

#include<iostream>
using namespace std;

namespace artery
{

using namespace omnetpp;

static const simsignal_t scSignalMcmReceived = cComponent::registerSignal("McmReceived");
static const simsignal_t scSignalMcmSent = cComponent::registerSignal("McmSent");

Define_Module(McService)

McService::McService() :
		mGenMcmMin { 100, SIMTIME_MS },
		mGenMcmMax { 1000, SIMTIME_MS },
		mGenMcm(mGenMcmMax),
		mGenMcmLowDynamicsCounter(0),
		mGenMcmLowDynamicsLimit(3)
{
}

void McService::initialize()
{
	ItsG5BaseService::initialize();
	mNetworkInterfaceTable = &getFacilities().get_const<NetworkInterfaceTable>();
	mVehicleDataProvider = &getFacilities().get_const<VehicleDataProvider>();
	mTimer = &getFacilities().get_const<Timer>();
	// mLocalDynamicMap = &getFacilities().get_mutable<artery::LocalDynamicMap>();

	// avoid unreasonable high elapsed time values for newly inserted vehicles
	mLastMcmTimestamp = simTime();

	// generation rate boundaries
	mGenMcmMin = par("minInterval");
	mGenMcmMax = par("maxInterval");
	mGenMcm = mGenMcmMax;

	// vehicle dynamics thresholds
	mHeadingDelta = vanetza::units::Angle { par("headingDelta").doubleValue() * vanetza::units::degree };
	mPositionDelta = par("positionDelta").doubleValue() * vanetza::units::si::meter;
	mSpeedDelta = par("speedDelta").doubleValue() * vanetza::units::si::meter_per_second;

	mDccRestriction = par("withDccRestriction");
	mFixedRate = par("fixedRate");

	// look up primary channel for MC
	mPrimaryChannel = getFacilities().get_const<MultiChannelPolicy>().primaryChannel(vanetza::aid::MC);
}

void McService::trigger()
{
	Enter_Method("trigger");
	checkTriggeringConditions(simTime());
}

void McService::indicate(const vanetza::btp::DataIndication& ind, std::unique_ptr<vanetza::UpPacket> packet)
{
	Enter_Method("indicate");

	Asn1PacketVisitor<vanetza::asn1::Mcm> visitor;
	const vanetza::asn1::Mcm* mcm = boost::apply_visitor(visitor, *packet);
	if (mcm && mcm->validate()) {
		McObject obj = visitor.shared_wrapper;
		emit(scSignalMcmReceived, &obj);
		// mLocalDynamicMap->updateAwareness(obj);
	}
}

void McService::checkTriggeringConditions(const SimTime& T_now)
{
	// provide variables named like in EN 302 637-2 V1.3.2 (section 6.1.3)
	SimTime& T_GenMcm = mGenMcm;
	const SimTime& T_GenMcmMin = mGenMcmMin;
	const SimTime& T_GenMcmMax = mGenMcmMax;
	const SimTime T_GenMcmDcc = mDccRestriction ? genMcmDcc() : T_GenMcmMin;
	const SimTime T_elapsed = T_now - mLastMcmTimestamp;

	if (T_elapsed >= T_GenMcmDcc) {
		if (mFixedRate) {
			sendMcm(T_now);
		} else if (checkHeadingDelta() || checkPositionDelta() || checkSpeedDelta()) {
			sendMcm(T_now);
			T_GenMcm = std::min(T_elapsed, T_GenMcmMax); /*< if middleware update interval is too long */
			mGenMcmLowDynamicsCounter = 0;
		} else if (T_elapsed >= T_GenMcm) {
			sendMcm(T_now);
			if (++mGenMcmLowDynamicsCounter >= mGenMcmLowDynamicsLimit) {
				T_GenMcm = T_GenMcmMax;
			}
		}
	}
}

bool McService::checkHeadingDelta() const
{
	return !vanetza::facilities::similar_heading(mLastMcmHeading, mVehicleDataProvider->heading(), mHeadingDelta);
}

bool McService::checkPositionDelta() const
{
	return (distance(mLastMcmPosition, mVehicleDataProvider->position()) > mPositionDelta);
}

bool McService::checkSpeedDelta() const
{
	return abs(mLastMcmSpeed - mVehicleDataProvider->speed()) > mSpeedDelta;
}

void McService::sendMcm(const SimTime& T_now)
{
	uint16_t genDeltaTimeMod = countTaiMilliseconds(mTimer->getTimeFor(mVehicleDataProvider->updated()));
	auto mcm = createManoeuvreCoordinationMessage(*mVehicleDataProvider, genDeltaTimeMod, 1, 1, 1, 1, 1, 1);

	mLastMcmPosition = mVehicleDataProvider->position();
	mLastMcmSpeed = mVehicleDataProvider->speed();
	mLastMcmHeading = mVehicleDataProvider->heading();
	mLastMcmTimestamp = T_now;

	using namespace vanetza;
	btp::DataRequestB request;
	request.destination_port = btp::ports::MCM;
	request.gn.its_aid = aid::MC;
	request.gn.transport_type = geonet::TransportType::SHB;
	request.gn.maximum_lifetime = geonet::Lifetime { geonet::Lifetime::Base::One_Second, 1 };
	request.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP2));
	request.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;

	McObject obj(std::move(mcm));
	emit(scSignalMcmSent, &obj);

	using McmByteBuffer = convertible::byte_buffer_impl<asn1::Mcm>;
	std::unique_ptr<geonet::DownPacket> payload { new geonet::DownPacket() };

	cout << "payload size: " << payload->size() << endl;

	std::unique_ptr<convertible::byte_buffer> buffer { new McmByteBuffer(obj.shared_ptr()) };

	cout << "buffer size: " << buffer->size() << endl;

	payload->layer(OsiLayer::Application) = std::move(buffer);

	cout << "payload size after move: " << payload->size() << endl;
	cout << "before request" << endl;

	this->request(request, std::move(payload));

	cout << "after request" << endl;
}

SimTime McService::genMcmDcc()
{
	// network interface may not be ready yet during initialization, so look it up at this later point
	auto netifc = mNetworkInterfaceTable->select(mPrimaryChannel);
	vanetza::dcc::TransmitRateThrottle* trc = netifc ? netifc->getDccEntity().getTransmitRateThrottle() : nullptr;
	if (!trc) {
		throw cRuntimeError("No DCC TRC found for MC's primary channel %i", mPrimaryChannel);
	}

	static const vanetza::dcc::TransmissionLite mc_tx(vanetza::dcc::Profile::DP2, 0);
	vanetza::Clock::duration interval = trc->interval(mc_tx);
	SimTime dcc { std::chrono::duration_cast<std::chrono::milliseconds>(interval).count(), SIMTIME_MS };
	return std::min(mGenMcmMax, std::max(mGenMcmMin, dcc));
}

vanetza::asn1::Mcm createManoeuvreCoordinationMessage(const VehicleDataProvider& vdp, uint16_t genDeltaTime, unsigned mcmTrajectoriesLength, unsigned intermediatePointsLength, unsigned longitudinalPositionsLength, unsigned longitudinalPositionsCoefficientsLength, unsigned lateralPositionsLength, unsigned lateralPositionsCoefficientsLength)
{
	cout << "MCM" << endl;
	
	vanetza::asn1::Mcm message;

	ItsPduHeader_t& header = (*message).header;
	header.protocolVersion = 2;
	header.messageID = ItsPduHeader__messageID_mcm;
	header.stationID = vdp.station_id();
	// cout << "header.protocolVersion: " << header.protocolVersion << endl;
	// cout << "header.messageID: " << header.messageID << endl;
	// cout << "header.stationID: " << header.stationID << endl;

	ManueverCoordination_t& mcm = (*message).mcm;
	mcm.generationDeltaTime = genDeltaTime * GenerationDeltaTime_oneMilliSec;
	mcm.mcmContainer.present = McmContainer_PR_vehicleManoeuvreContainer;
	// cout << "mcm.generationDeltaTime: " << mcm.generationDeltaTime << endl;
	// cout << "mcm.mcmContainer.present: " << mcm.mcmContainer.present << endl;

	VehicleManoeuvreContainer_t& vmc = mcm.mcmContainer.choice.vehicleManoeuvreContainer;
	vmc.currentPoint.present = McmStartPoint_PR_intermediatePointReference;
	// cout << "mcm.mcmContainer.vehicleManoeuvreContainer.currentPoint.present: " << vmc.currentPoint.present << endl;


	IntermediatePointReference_t& ipr = vmc.currentPoint.choice.intermediatePointReference;
	ipr.referencePosition.latitude = Latitude_unavailable;
	ipr.referencePosition.longitude = Longitude_unavailable;
	ipr.referencePosition.positionConfidenceEllipse.semiMajorConfidence = SemiAxisLength_unavailable;
	ipr.referencePosition.positionConfidenceEllipse.semiMinorConfidence = SemiAxisLength_unavailable;
	ipr.referencePosition.positionConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;
	ipr.referencePosition.altitude.altitudeValue = AltitudeValue_unavailable;
	ipr.referencePosition.altitude.altitudeConfidence = AltitudeConfidence_unavailable;
	ipr.referenceHeading.headingValue = HeadingValue_unavailable;
	ipr.referenceHeading.headingConfidence = HeadingConfidence_unavailable;
	ipr.lane.lanePosition = LanePosition_innerHardShoulder;
	ipr.lane.laneCount = 2; // Number of Lanes (INTEGER (1..16))
	ipr.timeOfPos = 1; // INTEGER (0..65535)
	// cout << "mcm.mcmContainer.vehicleManoeuvreContainer.currentPoint.intermediatePointReference.referencePosition.latitude: " << ipr.referencePosition.latitude << endl;
	// cout << "mcm.mcmContainer.vehicleManoeuvreContainer.currentPoint.intermediatePointReference.referencePosition.longitude: " << ipr.referencePosition.longitude << endl;
	// cout << "mcm.mcmContainer.vehicleManoeuvreContainer.currentPoint.intermediatePointReference.referencePosition.positionConfidenceEllipse.semiMajorConfidence: " << ipr.referencePosition.positionConfidenceEllipse.semiMajorConfidence << endl;
	// cout << "mcm.mcmContainer.vehicleManoeuvreContainer.currentPoint.intermediatePointReference.referencePosition.positionConfidenceEllipse.semiMinorConfidence: " << ipr.referencePosition.positionConfidenceEllipse.semiMinorConfidence << endl;
	// cout << "mcm.mcmContainer.vehicleManoeuvreContainer.currentPoint.intermediatePointReference.referencePosition.positionConfidenceEllipse.semiMajorOrientation: " << ipr.referencePosition.positionConfidenceEllipse.semiMajorOrientation << endl;
	// cout << "mcm.mcmContainer.vehicleManoeuvreContainer.currentPoint.intermediatePointReference.referencePosition.altitude.altitudeValue: " << ipr.referencePosition.altitude.altitudeValue << endl;
	// cout << "mcm.mcmContainer.vehicleManoeuvreContainer.currentPoint.intermediatePointReference.referencePosition.altitude.altitudeConfidence: " << ipr.referencePosition.altitude.altitudeConfidence << endl;
	// cout << "mcm.mcmContainer.vehicleManoeuvreContainer.currentPoint.intermediatePointReference.referenceHeading.headingValue: " << ipr.referenceHeading.headingValue << endl;
	// cout << "mcm.mcmContainer.vehicleManoeuvreContainer.currentPoint.intermediatePointReference.referenceHeading.headingConfidence: " << ipr.referenceHeading.headingConfidence << endl;
	// cout << "mcm.mcmContainer.vehicleManoeuvreContainer.currentPoint.intermediatePointReference.lane.lanePosition: " << ipr.lane.lanePosition << endl;
	// cout << "mcm.mcmContainer.vehicleManoeuvreContainer.currentPoint.intermediatePointReference.lane.laneCount: " << ipr.lane.laneCount << endl;
	// cout << "mcm.mcmContainer.vehicleManoeuvreContainer.currentPoint.intermediatePointReference.timeOfPos: " << ipr.timeOfPos << endl;

	// vmc.mcmTrajectories (1..16)
	if (mcmTrajectoriesLength > 16) {
		EV_WARN << "mcmTrajectories can contain 16 elements at maximum";
		// cout << "mcmTrajectories can contain 16 elements at maximum" << endl;
		mcmTrajectoriesLength = 16;
	}
	for (unsigned i = 0; i < mcmTrajectoriesLength; ++i) {
		McmTrajectory* t = vanetza::asn1::allocate<McmTrajectory>();
		t->trajectoryID = i; // INTEGER (0..65535)
		// cout << "mcm.mcmContainer.vehicleManoeuvreContainer.mcmTrajectories[" << i << "].trajectoryID: " << t->trajectoryID << endl;
		// t->trajectory
		// t->trajectory.intermediatePoints (1..10)
		if (intermediatePointsLength > 10) {
			EV_WARN << "intermediatePoints can contain 10 elements at maximum";
			// cout << "intermediatePoints can contain 10 elements at maximum" << endl;
			intermediatePointsLength = 10;
		}
		for (unsigned j = 0; j < intermediatePointsLength; j++) {
			IntermediatePoint* ip = vanetza::asn1::allocate<IntermediatePoint>();
			ip->present = IntermediatePoint_PR_reference;
			// cout << "mcm.mcmContainer.vehicleManoeuvreContainer.mcmTrajectories[" << i << "].trajectory.intermediatePoints[" << j << "].present: " << ip->present << endl;
			IntermediatePointReference_t& ref = ip->choice.reference;
			ref.referencePosition.latitude = Latitude_unavailable;
			ref.referencePosition.longitude = Longitude_unavailable;
			ref.referencePosition.positionConfidenceEllipse.semiMajorConfidence = SemiAxisLength_unavailable;
			ref.referencePosition.positionConfidenceEllipse.semiMinorConfidence = SemiAxisLength_unavailable;
			ref.referencePosition.positionConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;
			ref.referencePosition.altitude.altitudeValue = AltitudeValue_unavailable;
			ref.referencePosition.altitude.altitudeConfidence = AltitudeConfidence_unavailable;
			ref.referenceHeading.headingValue = HeadingValue_unavailable;
			ref.referenceHeading.headingConfidence = HeadingConfidence_unavailable;
			ref.lane.lanePosition = LanePosition_innerHardShoulder;
			ref.lane.laneCount = 2; // Number of Lanes (INTEGER (1..16))
			ref.timeOfPos = j; // INTEGER (0..65535)
			// cout << "mcm.mcmContainer.vehicleManoeuvreContainer.mcmTrajectories[" << i << "].trajectory.intermediatePoints[" << j << "].reference.referencePosition.latitude: " << ref.referencePosition.latitude << endl;
			// cout << "mcm.mcmContainer.vehicleManoeuvreContainer.mcmTrajectories[" << i << "].trajectory.intermediatePoints[" << j << "].reference.referencePosition.longitude: " << ref.referencePosition.longitude << endl;
			// cout << "mcm.mcmContainer.vehicleManoeuvreContainer.mcmTrajectories[" << i << "].trajectory.intermediatePoints[" << j << "].reference.referencePosition.positionConfidenceEllipse.semiMajorConfidence: " << ref.referencePosition.positionConfidenceEllipse.semiMajorConfidence << endl;
			// cout << "mcm.mcmContainer.vehicleManoeuvreContainer.mcmTrajectories[" << i << "].trajectory.intermediatePoints[" << j << "].reference.referencePosition.positionConfidenceEllipse.semiMinorConfidence: " << ref.referencePosition.positionConfidenceEllipse.semiMinorConfidence << endl;
			// cout << "mcm.mcmContainer.vehicleManoeuvreContainer.mcmTrajectories[" << i << "].trajectory.intermediatePoints[" << j << "].reference.referencePosition.positionConfidenceEllipse.semiMajorOrientation: " << ref.referencePosition.positionConfidenceEllipse.semiMajorOrientation << endl;
			// cout << "mcm.mcmContainer.vehicleManoeuvreContainer.mcmTrajectories[" << i << "].trajectory.intermediatePoints[" << j << "].reference.referencePosition.altitude.altitudeValue: " << ref.referencePosition.altitude.altitudeValue << endl;
			// cout << "mcm.mcmContainer.vehicleManoeuvreContainer.mcmTrajectories[" << i << "].trajectory.intermediatePoints[" << j << "].reference.referencePosition.altitude.altitudeConfidence: " << ref.referencePosition.altitude.altitudeConfidence << endl;
			// cout << "mcm.mcmContainer.vehicleManoeuvreContainer.mcmTrajectories[" << i << "].trajectory.intermediatePoints[" << j << "].reference.referenceHeading.headingValue: " << ref.referenceHeading.headingValue << endl;
			// cout << "mcm.mcmContainer.vehicleManoeuvreContainer.mcmTrajectories[" << i << "].trajectory.intermediatePoints[" << j << "].reference.referenceHeading.headingConfidence: " << ref.referenceHeading.headingConfidence << endl;
			// cout << "mcm.mcmContainer.vehicleManoeuvreContainer.mcmTrajectories[" << i << "].trajectory.intermediatePoints[" << j << "].reference.lane.lanePosition: " << ref.lane.lanePosition << endl;
			// cout << "mcm.mcmContainer.vehicleManoeuvreContainer.mcmTrajectories[" << i << "].trajectory.intermediatePoints[" << j << "].reference.lane.laneCount: " << ref.lane.laneCount << endl;
			// cout << "mcm.mcmContainer.vehicleManoeuvreContainer.mcmTrajectories[" << i << "].trajectory.intermediatePoints[" << j << "].reference.timeOfPos: " << ref.timeOfPos << endl;
			ASN_SEQUENCE_ADD(&t->trajectory.intermediatePoints, ip);
		}
		// t->trajectory.longitudinalPositions (1..11)
		if (longitudinalPositionsLength > 11) {
			EV_WARN << "longitudinalPositions can contain 11 elements at maximum";
			// cout << "longitudinalPositions can contain 11 elements at maximum" << endl;
			longitudinalPositionsLength = 11;
		}
		for (unsigned j = 0; j < longitudinalPositionsLength; j++) {
			Polynom* p = vanetza::asn1::allocate<Polynom>();
			// p->coefficients (1..6)
			if (longitudinalPositionsCoefficientsLength > 6) {
				EV_WARN << "longitudinalPositionsCoefficients can contain 6 elements at maximum";
				// cout << "longitudinalPositionsCoefficients can contain 6 elements at maximum" << endl;
				longitudinalPositionsCoefficientsLength = 6;
			}
			for (unsigned k = 0; k < longitudinalPositionsCoefficientsLength; k++) {
				double* c = (double*)1;
				// cout << "mcm.mcmContainer.vehicleManoeuvreContainer.mcmTrajectories[" << i << "].trajectory.longitudinalPositions[" << j << "].coefficients[" << c << "]: " << c << endl;
				ASN_SEQUENCE_ADD(&p->coefficients, c);
			}
			p->start = j; // INTEGER (0..2097151)
			p->end = j+1; // INTEGER (0..2097151)
			p->xOffset = p->start - p->end; // INTEGER (-8000000..8000000)
			// cout << "mcm.mcmContainer.vehicleManoeuvreContainer.mcmTrajectories[" << i << "].trajectory.longitudinalPositions[" << j << "].start: " << p->start<< endl;
			// cout << "mcm.mcmContainer.vehicleManoeuvreContainer.mcmTrajectories[" << i << "].trajectory.longitudinalPositions[" << j << "].end: " << p->end << endl;
			// cout << "mcm.mcmContainer.vehicleManoeuvreContainer.mcmTrajectories[" << i << "].trajectory.longitudinalPositions[" << j << "].xOffset: " << p->xOffset << endl;
			ASN_SEQUENCE_ADD(&t->trajectory.longitudinalPositions, p);
		}
		// t->trajectory.lateralPositions (1..11)
		if (lateralPositionsLength > 11) {
			EV_WARN << "lateralPositions can contain 11 elements at maximum";
			// cout << "lateralPositions can contain 11 elements at maximum" << endl;
			lateralPositionsLength = 11;
		}
		for (unsigned j = 0; j < lateralPositionsLength; j++) {
			Polynom* p = vanetza::asn1::allocate<Polynom>();
			// p->coefficients (1..6)
			if (lateralPositionsCoefficientsLength > 6) {
				EV_WARN << "lateralPositionsCoefficients can contain 6 elements at maximum";
				// cout << "lateralPositionsCoefficients can contain 6 elements at maximum" << endl;
				lateralPositionsCoefficientsLength = 6;
			}
			for (unsigned k = 0; k < lateralPositionsCoefficientsLength; k++) {
				double* c = (double*)1;
				// cout << "mcm.mcmContainer.vehicleManoeuvreContainer.mcmTrajectories[" << i << "].trajectory.lateralPositions[" << j << "].coefficients[" << c << "]: " << c << endl;
				ASN_SEQUENCE_ADD(&p->coefficients, c);
			}
			p->start = j; // INTEGER (0..2097151)
			p->end = j+1; // INTEGER (0..2097151)
			p->xOffset = p->start - p->end;
			// cout << "mcm.mcmContainer.vehicleManoeuvreContainer.mcmTrajectories[" << i << "].trajectory.lateralPositions[" << j << "].start: " << p->start<< endl;
			// cout << "mcm.mcmContainer.vehicleManoeuvreContainer.mcmTrajectories[" << i << "].trajectory.lateralPositions[" << j << "].end: " << p->end << endl;
			// cout << "mcm.mcmContainer.vehicleManoeuvreContainer.mcmTrajectories[" << i << "].trajectory.lateralPositions[" << j << "].xOffset: " << p->xOffset << endl;
			ASN_SEQUENCE_ADD(&t->trajectory.lateralPositions, p);
		}
		t->cost = CooperationCost_zero;
		// cout << "mcm.mcmContainer.vehicleManoeuvreContainer.mcmTrajectories[" << i << "].cost: " << t->cost << endl;
		ASN_SEQUENCE_ADD(&vmc.mcmTrajectories, t);
	}

	std::string error;
	if (!message.validate(error)) {
		throw cRuntimeError("Invalid MCM: %s", error.c_str());
	}

	// cout << endl; 

	return message;
}
} // namespace artery