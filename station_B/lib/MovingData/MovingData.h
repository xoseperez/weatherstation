/*

  MovingData store and calculations
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

#ifndef _MOVING_DATA_h
#define _MOVING_DATA_h

class MovingData {

    private:

        float * _data;
        int _pointer;
        int _size;
        float _sum;
        int _count;

    public:

        MovingData(int size);
        ~MovingData();
        void reset();
        void store(float value);
        float average();
        float softAverage();
        float maximum();
        float minimum();
        float sum();
        float count();

};

#endif
