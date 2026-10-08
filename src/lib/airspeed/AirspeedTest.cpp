#include <gtest/gtest.h>
#include <math.h>

#include "airspeed.h"

static constexpr float kRhoSL = 1.225f;
static constexpr float kPressSL = 101325.f;

TEST(Airspeed, IASZeroPressureIsZero)
{
	EXPECT_FLOAT_EQ(calc_IAS(0.f), 0.f);
}

TEST(Airspeed, IASMatchesBernoulli)
{
	// v = sqrt(2 * dp / rho): 100 Pa -> ~12.78 m/s
	EXPECT_NEAR(calc_IAS(100.f), sqrtf(2.f * 100.f / kRhoSL), 1e-4f);
}

TEST(Airspeed, IASIsSignedForNegativePressure)
{
	EXPECT_NEAR(calc_IAS(-100.f), -calc_IAS(100.f), 1e-5f);
}

TEST(Airspeed, CASFromIASAppliesScale)
{
	EXPECT_FLOAT_EQ(calc_CAS_from_IAS(20.f, 1.1f), 22.f);
}

TEST(Airspeed, TASEqualsCASAtSeaLevelStandardDay)
{
	EXPECT_NEAR(calc_TAS_from_CAS(20.f, kPressSL, 15.f), 20.f, 0.05f);
}

TEST(Airspeed, TASGreaterThanCASAtAltitude)
{
	// ~3000 m: 70100 Pa, -4.5 C, thinner air means higher true airspeed
	EXPECT_GT(calc_TAS_from_CAS(20.f, 70100.f, -4.5f), 20.f);
}

TEST(Airspeed, TASFromCASFallsBackToStandardTempOnNaN)
{
	EXPECT_FLOAT_EQ(calc_TAS_from_CAS(20.f, kPressSL, NAN), calc_TAS_from_CAS(20.f, kPressSL, 15.f));
}

TEST(Airspeed, TrueCalibratedRoundTrip)
{
	const float rho = 0.9f;
	const float tas = calc_true_from_calibrated_airspeed(25.f, rho);
	EXPECT_NEAR(calc_calibrated_from_true_airspeed(tas, rho), 25.f, 1e-4f);
}

TEST(Airspeed, TASFromPressuresIsSignedAndFinite)
{
	EXPECT_GT(calc_TAS(kPressSL + 200.f, kPressSL, 15.f), 0.f);
	EXPECT_LT(calc_TAS(kPressSL - 200.f, kPressSL, 15.f), 0.f);
	// Invalid static pressure falls back to sea-level density instead of NaN
	EXPECT_TRUE(isfinite(calc_TAS(200.f, 0.f, 15.f)));
}

TEST(Airspeed, CorrectedMembraneEqualsUncorrected)
{
	EXPECT_NEAR(calc_IAS_corrected(AIRSPEED_COMPENSATION_MODEL_PITOT, AIRSPEED_SENSOR_MODEL_MEMBRANE,
				       0.2f, 1.5f, 100.f, kPressSL, 15.f), calc_IAS(100.f), 1e-4f);
}

TEST(Airspeed, CorrectedSDP3XAddsTubeLossCompensation)
{
	const float raw = calc_IAS(100.f);
	const float corrected = calc_IAS_corrected(AIRSPEED_COMPENSATION_MODEL_PITOT, AIRSPEED_SENSOR_MODEL_SDP3X,
				0.2f, 1.5f, 100.f, kPressSL, 15.f);
	EXPECT_GT(corrected, raw);
	EXPECT_LT(corrected, raw * 1.5f);
}

TEST(Airspeed, CorrectedKeepsSign)
{
	EXPECT_LT(calc_IAS_corrected(AIRSPEED_COMPENSATION_TUBE_PRESSURE_LOSS, AIRSPEED_SENSOR_MODEL_SDP3X,
				     0.2f, 1.5f, -100.f, kPressSL, 15.f), 0.f);
}
