#pragma once

/**
 * @brief Weighted average of two pressure readings.
 *
 * @note perc1 + perc2 must equal 1.0f.
 *
 * @param[in] val1  First pressure value (Pa).
 * @param[in] perc1 Weight of first value (0.0 - 1.0).
 * @param[in] val2  Second pressure value (Pa).
 * @param[in] perc2 Weight of second value (0.0 - 1.0).
 * @return float    Weighted average.
 */

float weighted_average(float val1, float perc1, float val2, float perc2);
