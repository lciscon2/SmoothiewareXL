/*
      this file is part of smoothie (http://smoothieware.org/). the motion control part is heavily based on grbl (https://github.com/simen/grbl).
      smoothie is free software: you can redistribute it and/or modify it under the terms of the gnu general public license as published by the free software foundation, either version 3 of the license, or (at your option) any later version.
      smoothie is distributed in the hope that it will be useful, but without any warranty; without even the implied warranty of merchantability or fitness for a particular purpose. see the gnu general public license for more details.
      you should have received a copy of the gnu general public license along with smoothie. if not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "Module.h"

#include <stdint.h>

class Tool : public Module
{
public:
    Tool(){};
    virtual ~Tool() {};

    virtual void select()= 0;
    virtual void deselect()= 0;
    virtual float *get_offset() { return offset; }
    virtual void set_offset(unsigned axis, float f) { if(axis < sizeof(offset)) offset[axis]= f; }
    virtual uint16_t get_name() const { return identifier; }

protected:
    float offset[3];
    uint16_t identifier;
};
