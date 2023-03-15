
package artery.application;

simple McService like ItsG5Service
{
    parameters:
        @signal[McmReceived](type=CaObject);
        @signal[McmSent](type=CaObject);

        @statistic[reception](source=McmReceived;record=vector(camStationId)?,vector(camGenerationDeltaTime)?);
        @statistic[transmission](source=McmSent;record=vector(camStationId)?,vector(camGenerationDeltaTime)?);

        // evaluate DCC transmission interval restrictions
        bool withDccRestriction = default(true);

        // generation interval boundaries
        double minInterval @unit(s) = default(0.1s);
        double maxInterval @unit(s) = default(1.0s);

        // generate at fixed rate (using minInterval, optionally restricted by DCC)
        bool fixedRate = default(false);

        // change in orientation triggering CAM generation (in degree)
        double headingDelta = default(4.0);

        // change in position triggering CAM generation
        double positionDelta @unit(m) = default(4.0m);

        // change in speed triggering CAM generation (in meter/second)
        double speedDelta @unit(mps) = default(0.5mps);

        // length of path history
        volatile int pathHistoryLength = default(23);
}