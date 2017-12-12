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
 *   Revision 1.8  2009-07-29 14:19:49  jhoffmann
 *   Various bugfixing, remove memory leaks (with valgrind->memcheck / alleyoop)
 *
 *   Revision 1.7  2008/05/07 16:45:52  martius
 *   code cosmetics and documentation
 *
 *   Revision 1.6  2007/08/23 15:39:05  martius
 *   new IR sensor schema which uses substances and callbacks, very nice
 *
 *   Revision 1.5  2007/04/03 14:12:24  der
 *   getSensorNumber
 *
 *   Revision 1.4  2006/09/20 12:56:28  martius
 *   setRange
 *
 *   Revision 1.3  2006/07/14 12:23:43  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.2.4.4  2006/01/12 15:14:57  martius
 *   indentation and clear routine
 *
 *   Revision 1.2.4.3  2005/12/14 15:37:19  martius
 *   sensors are working with osg
 *
 *   Revision 1.2.4.2  2005/12/14 12:43:07  martius
 *   moved to osg
 *
 *   Revision 1.2.4.1  2005/12/13 18:11:53  martius
 *   sensors ported, but not yet finished
 *
 *   Revision 1.2  2005/09/27 13:59:26  martius
 *   ir sensors are working now
 *
 *   Revision 1.1  2005/09/27 11:03:34  fhesse
 *   sensorbank added
 *
 *                                                                         *
>>>>>>> parent of c757c4e1... renamed globally ode to ode-dbl
 ***************************************************************************/

#include <assert.h>
#include <ode/ode.h>
#include <osg/Matrix>

#include "raysensorbank.h"

using namespace osg;

namespace lpzrobots {

  RaySensorBank::RaySensorBank() : initialized(false)
  {  };

  RaySensorBank::~RaySensorBank()
  {
    clear();

    // if (initialized)
      //      this->odeHandle.deleteSpace(); this is automatically done if the parent space is deleted

    initialized = false;
  };

  void RaySensorBank::setInitData(const OdeHandle& odeHandle,
                                  const OsgHandle& osgHandle,
                                  const osg::Matrix& pose) {
    PhysicalSensor::setInitData(odeHandle, osgHandle, pose);
    this->odeHandle.createNewSimpleSpace(odeHandle.space, true);
  }


  void RaySensorBank::init(Primitive* own, Joint* joint ) {
    initialized=true;
  };

  unsigned int RaySensorBank::registerSensor(RaySensor* raysensor, Primitive* body,
                                             const osg::Matrix& pose, float range,
                                             RaySensor::rayDrawMode drawMode){
    assert(isInitDataSet);
    raysensor->setDrawMode(drawMode);
    raysensor->setRange(range);
    raysensor->setInitData(odeHandle, osgHandle, pose);
    raysensor->init(body);
    bank.push_back(raysensor);
    return bank.size();
  };

  bool RaySensorBank::sense(const GlobalData& global){
    for (unsigned int i=0; i<bank.size(); i++) {
        bank[i]->sense(global);
    }
    return true;
  };

  int RaySensorBank::get(double* sensorarray, int array_size) const {
    int counter=0;
    for(int i=0; (i<array_size) && (i<(int)bank.size()); i++){
      bank[i]->get(&sensorarray[i], 1);
      counter++;
    }
    return counter;
  };

  std::list<sensor> RaySensorBank::getList() const {
    return getListOfArray();
  }

  int RaySensorBank::getSensorNumber() const {
    return bank.size();
  }

  void RaySensorBank::setRange(unsigned int index, float range){
    assert(index<bank.size());
    return bank[index]->setRange(range);
  }

  void RaySensorBank::setRange(float range){
    for(unsigned int i=0; i<bank.size(); i++){
      bank[i]->setRange(range);
    }
  }


  dSpaceID RaySensorBank::getSpaceID(){
    return odeHandle.space;
  };

  void RaySensorBank::update(){
    for (unsigned int i=0; i<bank.size(); i++){
      bank[i]->update();
    }
  };

  // delete all registered sensors.
  void RaySensorBank::clear(){
    for (unsigned int i=0; i<bank.size(); i++)
    {
      if(bank[i])
        delete bank[i];
    }
    bank.clear();
    //odeHandle.deleteSpace();
  }


}





