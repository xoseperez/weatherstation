/*

  Moving data (moving average, sum...) store and calculations
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
#include "MovingData.h"

MovingData::MovingData(int size) {
    _size = size;
    _data = (float *) malloc(_size * sizeof(float));
    reset();
}

MovingData::~MovingData() {
    free(_data);
}

void MovingData::reset() {
    _sum = 0;
    _count = 0;
    _pointer = 0;
    for(int i = 0; i < _size; i++) _data[i] = 0.0;
}

/*
 * record
 * Records the value into a measure structure
 *
 * @param measure magnitude
 * @param float value
 * @return void
 */
void MovingData::store(float value) {
    _sum = _sum - _data[_pointer] + value;
    _data[_pointer] = value;
    ++_pointer;
    if (_count < _size) ++_count;
    if (_pointer == _size) _pointer = 0;
}

/*
 * average
 * Calculates the average for the stored values
 *
 * @return float
 */
float MovingData::average() {
    if (_count == 0) return 0.0;
    return _sum / _count;
}

/*
 * softAverage
 * Calculates the average for the stored values substracting the minimum and maximum first
 *
 * @return float
 */
float MovingData::softAverage() {
    if (_count < 3) return 0.0;
    return (_sum - minimum() - maximum()) / (_count - 2);
}

float MovingData::count() {
    return _count;
}

float MovingData::sum() {
    return _sum;
}

float MovingData::minimum() {
    float minimum = 0.0;
    if (_count > 0) {
        minimum = _data[0];
        for (int i=1; i<_count; i++) {
            minimum = min(minimum, _data[i]);
        }
    }
    return minimum;
}

float MovingData::maximum() {
    float maximum = 0.0;
    if (_count > 0) {
        maximum = _data[0];
        for (int i=1; i<_count; i++) {
            maximum = max(maximum, _data[i]);
        }
    }
    return maximum;
}
