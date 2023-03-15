
#include "artery/application/CaObject.h"
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
#include <vanetza/facilities/cam_functions.hpp>
#include <chrono>

#include<iostream>
using namespace std;

namespace artery
{

using namespace omnetpp;

auto microdegree = vanetza::units::degree * boost::units::si::micro;
auto decidegree = vanetza::units::degree * boost::units::si::deci;
auto degree_per_second = vanetza::units::degree / vanetza::units::si::second;
auto centimeter_per_second = vanetza::units::si::meter_per_second * boost::units::si::centi;

static const simsignal_t scSignalMcmReceived = cComponent::registerSignal("McmReceived");
static const simsignal_t scSignalMcmSent = cComponent::registerSignal("McmSent");
static const auto scLowFrequencyContainerInterval = std::chrono::milliseconds(500);

template<typename T, typename U>
long round(const boost::units::quantity<T>& q, const U& u)
{
	boost::units::quantity<U> v { q };
	return std::round(v.value());
}

SpeedValue_t buildSpeedValue(const vanetza::units::Velocity& v)
{
	static const vanetza::units::Velocity lower { 0.0 * boost::units::si::meter_per_second };
	static const vanetza::units::Velocity upper { 163.82 * boost::units::si::meter_per_second };

	SpeedValue_t speed = SpeedValue_unavailable;
	if (v >= upper) {
		speed = 16382; // see CDD A.74 (TS 102 894 v1.2.1)
	} else if (v >= lower) {
		speed = round(v, centimeter_per_second) * SpeedValue_oneCentimeterPerSec;
	}
	return speed;
}


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
	mLocalDynamicMap = &getFacilities().get_mutable<artery::LocalDynamicMap>();

	// avoid unreasonable high elapsed time values for newly inserted vehicles
	mLastMcmTimestamp = simTime();

	// // first generated CAM shall include the low frequency container
	// mLastLowMcmTimestamp = mLastMcmTimestamp - artery::simtime_cast(scLowFrequencyContainerInterval);

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

	// look up primary channel for CA
	mPrimaryChannel = getFacilities().get_const<MultiChannelPolicy>().primaryChannel(vanetza::aid::CA);
}

void McService::trigger()
{
	Enter_Method("trigger");
	checkTriggeringConditions(simTime());
}

void McService::indicate(const vanetza::btp::DataIndication& ind, std::unique_ptr<vanetza::UpPacket> packet)
{
	Enter_Method("indicate");

	Asn1PacketVisitor<vanetza::asn1::Cam> visitor;
	const vanetza::asn1::Cam* mcm = boost::apply_visitor(visitor, *packet);
	if (mcm && mcm->validate()) {
		CaObject obj = visitor.shared_wrapper;
		emit(scSignalMcmReceived, &obj);
		mLocalDynamicMap->updateAwareness(obj);
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
	auto mcm = createManoeuvreCoordinationMessage(*mVehicleDataProvider, genDeltaTimeMod);

	mLastMcmPosition = mVehicleDataProvider->position();
	mLastMcmSpeed = mVehicleDataProvider->speed();
	mLastMcmHeading = mVehicleDataProvider->heading();
	mLastMcmTimestamp = T_now;
	// if (T_now - mLastLowMcmTimestamp >= artery::simtime_cast(scLowFrequencyContainerInterval)) {
	// 	addLowFrequencyContainer(mcm, par("pathHistoryLength"));
	// 	mLastLowMcmTimestamp = T_now;
	// }
    // 
	cout << endl; // MCM print end
	// 

	using namespace vanetza;
	btp::DataRequestB request;
	request.destination_port = btp::ports::MCM;
	request.gn.its_aid = aid::CA;
	request.gn.transport_type = geonet::TransportType::SHB;
	request.gn.maximum_lifetime = geonet::Lifetime { geonet::Lifetime::Base::One_Second, 1 };
	request.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP2));
	request.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;

	CaObject obj(std::move(mcm));
	emit(scSignalMcmSent, &obj);

	using CamByteBuffer = convertible::byte_buffer_impl<asn1::Cam>;
	std::unique_ptr<geonet::DownPacket> payload { new geonet::DownPacket() };
	std::unique_ptr<convertible::byte_buffer> buffer { new CamByteBuffer(obj.shared_ptr()) };
	payload->layer(OsiLayer::Application) = std::move(buffer);
	this->request(request, std::move(payload));
}

SimTime McService::genMcmDcc()
{
	// network interface may not be ready yet during initialization, so look it up at this later point
	auto netifc = mNetworkInterfaceTable->select(mPrimaryChannel);
	vanetza::dcc::TransmitRateThrottle* trc = netifc ? netifc->getDccEntity().getTransmitRateThrottle() : nullptr;
	if (!trc) {
		throw cRuntimeError("No DCC TRC found for CA's primary channel %i", mPrimaryChannel);
	}

	static const vanetza::dcc::TransmissionLite ca_tx(vanetza::dcc::Profile::DP2, 0);
	vanetza::Clock::duration interval = trc->interval(ca_tx);
	SimTime dcc { std::chrono::duration_cast<std::chrono::milliseconds>(interval).count(), SIMTIME_MS };
	return std::min(mGenMcmMax, std::max(mGenMcmMin, dcc));
}

vanetza::asn1::Cam createManoeuvreCoordinationMessage(const VehicleDataProvider& vdp, uint16_t genDeltaTime)
{
	vanetza::asn1::Cam message;

	ItsPduHeader_t& header = (*message).header;
	header.protocolVersion = 2;
	header.messageID = ItsPduHeader__messageID_mcm;
	header.stationID = vdp.station_id();
	// if (header.stationID==157) { // 3º vehicle
		// addSpecialVehicleContainer()
	// }

	CoopAwareness_t& mcm = (*message).cam;
	mcm.generationDeltaTime = genDeltaTime * GenerationDeltaTime_oneMilliSec;
	BasicContainer_t& basic = mcm.camParameters.basicContainer;
	HighFrequencyContainer_t& hfc = mcm.camParameters.highFrequencyContainer;

	basic.stationType = StationType_passengerCar;
	basic.referencePosition.altitude.altitudeValue = AltitudeValue_unavailable;
	basic.referencePosition.altitude.altitudeConfidence = AltitudeConfidence_unavailable;
	basic.referencePosition.longitude = round(vdp.longitude(), microdegree) * Longitude_oneMicrodegreeEast;
	basic.referencePosition.latitude = round(vdp.latitude(), microdegree) * Latitude_oneMicrodegreeNorth;
	basic.referencePosition.positionConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;
	basic.referencePosition.positionConfidenceEllipse.semiMajorConfidence =
			SemiAxisLength_unavailable;
	basic.referencePosition.positionConfidenceEllipse.semiMinorConfidence =
			SemiAxisLength_unavailable;

	hfc.present = HighFrequencyContainer_PR_basicVehicleContainerHighFrequency;
	BasicVehicleContainerHighFrequency& bvc = hfc.choice.basicVehicleContainerHighFrequency;
	bvc.heading.headingValue = round(vdp.heading(), decidegree);
	bvc.heading.headingConfidence = HeadingConfidence_equalOrWithinOneDegree;
	bvc.speed.speedValue = buildSpeedValue(vdp.speed());
	bvc.speed.speedConfidence = SpeedConfidence_equalOrWithinOneCentimeterPerSec * 3;
	bvc.driveDirection = vdp.speed().value() >= 0.0 ?
			DriveDirection_forward : DriveDirection_backward;
	const double lonAccelValue = vdp.acceleration() / vanetza::units::si::meter_per_second_squared;
	// extreme speed changes can occur when SUMO swaps vehicles between lanes (speed is swapped as well)
	if (lonAccelValue >= -160.0 && lonAccelValue <= 161.0) {
		bvc.longitudinalAcceleration.longitudinalAccelerationValue = lonAccelValue * LongitudinalAccelerationValue_pointOneMeterPerSecSquaredForward;
	} else {
		bvc.longitudinalAcceleration.longitudinalAccelerationValue = LongitudinalAccelerationValue_unavailable;
	}
	bvc.longitudinalAcceleration.longitudinalAccelerationConfidence = AccelerationConfidence_unavailable;
	bvc.curvature.curvatureValue = abs(vdp.curvature() / vanetza::units::reciprocal_metre) * 10000.0;
	if (bvc.curvature.curvatureValue >= 1023) {
		bvc.curvature.curvatureValue = 1023;
	}
	bvc.curvature.curvatureConfidence = CurvatureConfidence_unavailable;
	bvc.curvatureCalculationMode = CurvatureCalculationMode_yawRateUsed;
	bvc.yawRate.yawRateValue = round(vdp.yaw_rate(), degree_per_second) * YawRateValue_degSec_000_01ToLeft * 100.0;
	if (bvc.yawRate.yawRateValue < -32766 || bvc.yawRate.yawRateValue > 32766) {
		bvc.yawRate.yawRateValue = YawRateValue_unavailable;
	}
	bvc.vehicleLength.vehicleLengthValue = VehicleLengthValue_unavailable;
	bvc.vehicleLength.vehicleLengthConfidenceIndication =
			VehicleLengthConfidenceIndication_noTrailerPresent;
	bvc.vehicleWidth = VehicleWidth_unavailable;

    // bvc.lanePosition = LanePosition_offTheRoad; // it changes the field once and gives an error because of SUMO, since the correct value sould be LanePosition_innerHardShoulder (0)

	std::string error;
	if (!message.validate(error)) {
		throw cRuntimeError("Invalid High Frequency CAM: %s", error.c_str());
	}

    //
    // MCM = CAM without lowFrequencyContainer (commented)
	cout << "MCM" << endl;
	cout << "header.protocolVersion: " << header.protocolVersion << endl;
	cout << "header.messageID: " << header.messageID << endl;
	cout << "header.stationID: " << header.stationID << endl;
	cout << "mcm.generationDeltaTime: " << mcm.generationDeltaTime << endl;
	cout << "basic.stationType: " << basic.stationType << endl;
	cout << "basic.referencePosition.altitude.altitudeValue: " << basic.referencePosition.altitude.altitudeValue << endl;
	cout << "basic.referencePosition.altitude.altitudeConfidence: " << basic.referencePosition.altitude.altitudeConfidence << endl;
	cout << "basic.referencePosition.longitude: " << basic.referencePosition.longitude << endl;
	cout << "basic.referencePosition.latitude: " << basic.referencePosition.latitude << endl;
	cout << "basic.referencePosition.positionConfidenceEllipse.semiMajorOrientation: " << basic.referencePosition.positionConfidenceEllipse.semiMajorOrientation << endl;
	cout << "basic.referencePosition.positionConfidenceEllipse.semiMajorConfidence: " << basic.referencePosition.positionConfidenceEllipse.semiMajorConfidence << endl;
	cout << "basic.referencePosition.positionConfidenceEllipse.semiMinorConfidence: " << basic.referencePosition.positionConfidenceEllipse.semiMinorConfidence << endl;
	cout << "hfc.present: " << hfc.present << endl;
	cout << "bvc.heading.headingValue: " << bvc.heading.headingValue << endl;
	cout << "bvc.heading.headingConfidence: " << bvc.heading.headingConfidence << endl;
	cout << "bvc.speed.speedValue: " << bvc.speed.speedValue << endl;
	cout << "bvc.speed.speedConfidence: " << bvc.speed.speedConfidence << endl;
	cout << "bvc.driveDirection: " << bvc.driveDirection << endl;
	cout << "bvc.vehicleLength.vehicleLengthValue: " << bvc.vehicleLength.vehicleLengthValue << endl;
	cout << "bvc.vehicleLength.vehicleLengthConfidenceIndication: " << bvc.vehicleLength.vehicleLengthConfidenceIndication << endl;
	cout << "bvc.vehicleWidth: " << bvc.vehicleWidth << endl;
	cout << "bvc.longitudinalAcceleration.longitudinalAccelerationValue: " << bvc.longitudinalAcceleration.longitudinalAccelerationValue << endl;
	cout << "bvc.longitudinalAcceleration.longitudinalAccelerationConfidence: " << bvc.longitudinalAcceleration.longitudinalAccelerationConfidence << endl;
	cout << "bvc.curvature.curvatureValue: " << bvc.curvature.curvatureValue << endl;
	cout << "bvc.curvature.curvatureConfidence: " << bvc.curvature.curvatureConfidence << endl;
	cout << "bvc.curvatureCalculationMode: " << bvc.curvatureCalculationMode << endl;
	cout << "bvc.yawRate.yawRateValue: " << bvc.yawRate.yawRateValue << endl;
	cout << "bvc.yawRate.yawRateConfidence: " << bvc.yawRate.yawRateConfidence << endl;
	cout << "bvc.lanePosition: " << bvc.lanePosition << endl;
	//

	return message;
}

// void addLowFrequencyContainer(vanetza::asn1::Cam& message, unsigned pathHistoryLength)
// {
// 	if (pathHistoryLength > 40) {
// 		EV_WARN << "path history can contain 40 elements at maximum";
// 		pathHistoryLength = 40;
// 	}

// 	LowFrequencyContainer_t*& lfc = message->cam.camParameters.lowFrequencyContainer;
// 	lfc = vanetza::asn1::allocate<LowFrequencyContainer_t>();
// 	lfc->present = LowFrequencyContainer_PR_basicVehicleContainerLowFrequency;
// 	BasicVehicleContainerLowFrequency& bvc = lfc->choice.basicVehicleContainerLowFrequency;
// 	bvc.vehicleRole = VehicleRole_default;
// 	bvc.exteriorLights.buf = static_cast<uint8_t*>(vanetza::asn1::allocate(1));
// 	assert(nullptr != bvc.exteriorLights.buf);
// 	bvc.exteriorLights.size = 1;
// 	bvc.exteriorLights.buf[0] |= 1 << (7 - ExteriorLights_daytimeRunningLightsOn);

// 	for (unsigned i = 0; i < pathHistoryLength; ++i) {
// 		PathPoint* pathPoint = vanetza::asn1::allocate<PathPoint>();
// 		pathPoint->pathDeltaTime = vanetza::asn1::allocate<PathDeltaTime_t>();
// 		*(pathPoint->pathDeltaTime) = (i + 1) * PathDeltaTime_tenMilliSecondsInPast * 10;
// 		pathPoint->pathPosition.deltaLatitude = DeltaLatitude_unavailable;
// 		pathPoint->pathPosition.deltaLongitude = DeltaLongitude_unavailable;
// 		pathPoint->pathPosition.deltaAltitude = DeltaAltitude_unavailable;
// 		ASN_SEQUENCE_ADD(&bvc.pathHistory, pathPoint);
// 	}

// 	std::string error;
// 	if (!message.validate(error)) {
// 		throw cRuntimeError("Invalid Low Frequency CAM: %s", error.c_str());
// 	}

//     //
// 	cout << "bvc.vehicleRole: " << bvc.vehicleRole << endl;
// 	cout << "bvc.exteriorLights.size: " << bvc.exteriorLights.size << endl;
// 	//
// }

} // namespace artery