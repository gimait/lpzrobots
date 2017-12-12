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
=======
 *   $Log$
 *   Revision 1.9  2007-09-06 18:48:29  martius
 *   clone function (a bit like a factory)
 *
 *   Revision 1.8  2007/08/23 15:39:05  martius
 *   new IR sensor schema which uses substances and callbacks, very nice
 *
 *   Revision 1.7  2006/09/20 12:56:28  martius
 *   setRange
 *
 *   Revision 1.6  2006/08/28 12:18:31  martius
 *   documentation
 *
 *   Revision 1.5  2006/08/08 17:03:27  martius
 *   new sensors model
 *
 *   Revision 1.4  2006/07/14 12:23:43  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.3.4.3  2006/01/31 15:46:16  martius
 *   virtual destructor
 *
 *   Revision 1.3.4.2  2005/12/13 18:11:53  martius
 *   sensors ported, but not yet finished
 *
 *   Revision 1.3.4.1  2005/11/14 17:37:21  martius
 *   moved to selforg
 *
 *   Revision 1.3  2005/09/27 13:59:26  martius
 *   ir sensors are working now
 *
 *   Revision 1.2  2005/09/27 11:03:34  fhesse
 *   sensorbank added
 *
 *   Revision 1.1  2005/09/22 12:56:47  martius
 *   ray based sensors
 *
 *                                                                         *
>>>>>>> parent of c757c4e1... renamed globally ode to ode-dbl
 ***************************************************************************/
#ifndef __RAYSENSOR_H
#define __RAYSENSOR_H

<<<<<<< HEAD
#include <ode-dbl/common.h>
#include <ode-dbl/ode.h>
#include <cmath>
#include <assert.h>
#include <selforg/position.h>
#include <osg/Matrix>
#include <osg/Vec3>

#include "primitive.h"
#include "osgprimitive.h"
=======
#include <ode/common.h>
>>>>>>> parent of c757c4e1... renamed globally ode to ode-dbl
#include "osgforwarddecl.h"
#include "odehandle.h"
#include "physicalsensor.h"


namespace lpzrobots {

  class OSGCylinder;
  class OSGBox;
  class Transform;
  class Ray;

/** Class for Ray-based sensors.
    This are sensors which are based on distance measurements using the ODE geom class Ray.
    The sensor value is obtained by collisions.
    See also RaySensorBank, which is an object for managing multiple ray sensors.
 */
  class RaySensor : public PhysicalSensor {
  public:
    enum rayDrawMode { drawNothing, drawRay, drawSensor, drawAll};

    RaySensor();

    /**
     * @param size size of sensor in simulation
     * @param range maximum range of the Ray sensor
     * @param drawMode draw mode of the sensor
     */
    RaySensor(double size , double range, rayDrawMode drawMode);

    ~RaySensor();

    ///Create a copy of this without initialization
    virtual RaySensor* clone() const;

    void setPose(const osg::Matrix& pose) override;

    void init(Primitive* own, Joint* joint = 0) override;

    bool sense(const GlobalData& globaldata) override;

    int get(sensor* sensors, int length) const override;

    std::list<sensor> getList() const override;

    virtual int getSensorNumber() const override;

    virtual void update() override;

    ///Set maximum range of ray
    virtual void setRange(double range);

    ///Set draw mode of ray
    virtual void setDrawMode(rayDrawMode drawMode);

    ///Set length of ray (needed for callback)
    void setLength(double len, long int time);

  protected:
    //Initialize variables of ray sensor. Should be called
    //by every constructor
    void defaultInit();

    double size; // size of graphical sensor
    double range; // max length
    rayDrawMode drawMode;

    double len;   // measured length
    double lastlen; //last measured length
    double detection;   // detected length (internally used)
    long lasttimeasked; // used to make sense return the same number if called two times in one timestep

    OSGCylinder* sensorBody;
    Transform* transform;
    Ray* ray;
    bool initialised;

  };

}

#endif
