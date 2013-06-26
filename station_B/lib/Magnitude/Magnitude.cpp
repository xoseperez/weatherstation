/*

  Magnitude store and calculations
  Copyright (C) 2013 by Xose Pérez <xose dot perez at gmail dot com>

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#include <Arduino.h>
#include "Magnitude.h"

Magnitude::Magnitude() {
    reset();
}

void Magnitude::reset() {
    _sum = 0;
    _count = 0;
}

/*
 * record
 * Records the value into a measure structure
 *
 * @param measure magnitude
 * @param float value
 * @return void
 */
void Magnitude::store(float value) {
    _sum += value;
    _count += 1;
    if (_count == 1) {
        _minimum = value;
        _maximum = value;
    } else {
        _minimum = min(_minimum, value);
        _maximum = max(_maximum, value);
    }
}

/*
 * average
 * Calculates the average for the stored values substracting the minimum and maximum first
 *
 * @return float
 */
float Magnitude::average() {
    if (_count > 2) {
        return (_sum - _minimum - _maximum) / (_count - 2);
    } else {
        return 0.0;
    }
}

float Magnitude::count() {
    return _count;
}

float Magnitude::sum() {
    return _sum;
}

float Magnitude::minimum() {
    return _minimum;
}

float Magnitude::maximum() {
    return _maximum;
}
