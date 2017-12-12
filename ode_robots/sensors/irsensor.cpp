/***************************************************************************
 *   Copyright (C) 2005-2011 LpzRobots development team                    *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 *                                                                         *
<<<<<<< HEAD
 ***************************************************************************/
=======
 *   $Log$
 *   Revision 1.13  2009-01-20 17:29:52  martius
 *   cvs commit
 *
 *   Revision 1.12  2008/09/16 14:53:59  martius
 *   use cmath instead of math.h
 *
 *   Revision 1.11  2007/09/06 18:48:29  martius
 *   clone function (a bit like a factory)
 *
 *   Revision 1.10  2007/08/24 12:48:04  martius
 *   tranformation for sensor body fixed
 *
 *   Revision 1.9  2007/08/23 15:39:05  martius
 *   new IR sensor schema which uses substances and callbacks, very nice
 *
 *   Revision 1.8  2007/04/03 16:32:21  der
 *   thicker drawing
 *
 *   Revision 1.7  2006/12/11 18:25:08  martius
 *   memory freed
 *
 *   Revision 1.6  2006/09/20 12:56:28  martius
 *   setRange
 *
 *   Revision 1.5  2006/07/14 12:23:43  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.4.4.5  2006/01/26 18:37:20  martius
 *   *** empty log message ***
 *
 *   Revision 1.4.4.4  2005/12/14 15:37:19  martius
 *   sensors are working with osg
 *
 *   Revision 1.4.4.3  2005/12/14 12:43:07  martius
 *   moved to osg
 *
 *   Revision 1.4.4.2  2005/12/13 18:11:53  martius
 *   sensors ported, but not yet finished
 *
 *   Revision 1.4.4.1  2005/11/14 17:37:20  martius
 *   moved to selforg
 *
 *   Revision 1.4  2005/11/08 11:34:31  martius
 *   geom is only enabled in sense function
 *   there is no external collision detection anymore
 *
 *   Revision 1.3  2005/09/27 13:59:26  martius
 *   ir sensors are working now
 *
 *   Revision 1.2  2005/09/27 11:03:33  fhesse
 *   sensorbank added
 *
 *   Revision 1.1  2005/09/22 12:56:47  martius
 *   ray based sensors
 *
 *                                                                         *
 ***************************************************************************/
#include <ode/ode.h>
#include <cmath>
#include <assert.h>
#include <selforg/position.h>
#include <osg/Matrix>
#include <osg/Vec3>

#include "primitive.h"
#include "osgprimitive.h"
>>>>>>> parent of c757c4e1... renamed globally ode to ode-dbl
#include "irsensor.h"

namespace lpzrobots {

  IRSensor::IRSensor(double exponent, double size, double range, rayDrawMode drawMode)
  : RaySensor(size, range, drawMode) {

    this->exponent = exponent;
    value = 0;
  }


  bool IRSensor::sense(const GlobalData& globaldata){
    RaySensor::sense(globaldata);
    value = characteritic(len);
    return true;
  }

  double IRSensor::getValue(){
    return value;
  }

  int IRSensor::get(sensor* sensors, int length) const {
    assert(length>0);
    sensors[0]=value;
    return 1;
  }

  std::list<sensor> IRSensor::getList() const {
    return {value};
  }

  double IRSensor::characteritic(double len){
    double v = (range - len)/range;
    return v < 0 ? 0 : pow(v, exponent);
  }

}
