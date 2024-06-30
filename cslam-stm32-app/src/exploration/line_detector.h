/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#ifndef LINE_DETECTOR_H_
#define LINE_DETECTOR_H_

/**
 * @brief Computes the slope of the line defined by the ToF measurements.
 *
 * @param array Array containing the ToF measurments from a ToF sensor.
 * @return Slope of the line. -10 is returned in case no line is detected.
 */
float line_detector(float *array, int n);

#endif
