/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

/* 
 * Base code for CS 251 Software Systems Lab 
 * Department of Computer Science and Engineering, IIT Bombay
 * 
 */

#ifndef _DOMINOS_HPP_
#define _DOMINOS_HPP_
#include "cs251_base.hpp"
#include "render.hpp"
 #ifdef __APPLE__
  #include <GLUT/glut.h>
#else
  #include "GL/freeglut.h"
#endif

#include <cstring>
using namespace std;

namespace cs251
{
  //! This is the class that sets up the Box2D simulation world
  //! Notice the public inheritance - why do we inherit the base_sim_t class?
  class dominos_t : public base_sim_t
  {
    bool forceon[4];
    b2Body* b_magnetic[2];
    b2Body* b_metallic[2];
    float k;
  public:
    dominos_t();
    void step(settings_t *settings);    
    void keyboard(unsigned char key) ;
    void mouse_down(const b2Vec2& p);
    static base_sim_t* create()
    {
      return new dominos_t;
    }
  };
}
  
#endif
