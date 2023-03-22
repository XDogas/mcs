
package artery.application;

simple McService like ItsG5Service
{
    parameters:
        @signal[McmReceived](type=McObject);
        @signal[McmSent](type=McObject);

        @statistic[reception](source=McmReceived;record=vector(mcmStationId)?,vector(mcmGenerationDeltaTime)?);
        @statistic[transmission](source=McmSent;record=vector(mcmStationId)?,vector(mcmGenerationDeltaTime)?);

        // evaluate DCC transmission interval restrictions
        bool withDccRestriction = default(true);

        // generation interval boundaries
        double minInterval @unit(s) = default(0.1s);
        double maxInterval @unit(s) = default(1.0s);

        // generate at fixed rate (using minInterval, optionally restricted by DCC)
        bool fixedRate = default(false);

        // change in orientation triggering MCM generation (in degree)
        double headingDelta = default(4.0);

        // change in position triggering MCM generation
        double positionDelta @unit(m) = default(4.0m);

        // change in speed triggering MCM generation (in meter/second)
        double speedDelta @unit(mps) = default(0.5mps);

        // length of path history
        volatile int pathHistoryLength = default(23);
}