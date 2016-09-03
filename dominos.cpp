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


#include "cs251_base.hpp"
#include "render.hpp"

#ifdef __APPLE__
	#include <GLUT/glut.h>
#else
	#include "GL/freeglut.h"
#endif

#include <cstring>
using namespace std;
#include <math.h>
#include "dominos.hpp"
#include <iostream>
#include <stdlib.h>
#include <time.h>
namespace cs251
{
	/**  The is the constructor 
	 * This is the documentation block for the constructor.
	 */ 
	
	dominos_t::dominos_t(){
		srand((unsigned)time(0));
		m_world->SetGravity(b2Vec2(0,0));
		k=10000;
		no_of_magnets=6;
		no_of_metals=1;
		magnets.resize(no_of_magnets);
		metals.resize(no_of_metals);
		forceon.resize(no_of_magnets,-1);
		pos_magnets.resize(no_of_magnets);
		b2BodyDef bodies;
		bodies.type=b2_staticBody;
		b2CircleShape circle_magnets;
		circle_magnets.m_radius=3;
		circle_magnets.m_p.Set(0,0);
		b2FixtureDef f1;
		f1.shape=&circle_magnets;
		f1.restitution=0.7f;
		f1.friction=0;
		for(unsigned int i=0;i<no_of_magnets;i++){
			pos_magnets[i]=b2Vec2(rand()%81-40,rand()%31+10);
			bodies.position.Set(pos_magnets[i].x,pos_magnets[i].y);
			magnets[i]=m_world->CreateBody(&bodies);
			magnets[i]->CreateFixture(&f1);
		}
		b2CircleShape circle_metals;
		circle_metals.m_radius=1;
		circle_metals.m_p.Set(0,0);
		f1.shape=&circle_metals;
		bodies.position.Set(rand()%40-20,40);
		bodies.type=b2_dynamicBody;
		metals[0]=m_world->CreateBody(&bodies);
		metals[0]->CreateFixture(&f1);
		b2BodyDef slider_def;
		slider_def.type=b2_kinematicBody;
		slider_def.position.Set(0,0);
		b2PolygonShape slider_shape;
		slider_shape.SetAsBox(3.5f,0.2f);
		b2FixtureDef f2;
		f2.shape=&slider_shape;
		slider=m_world->CreateBody(&slider_def);
		slider->CreateFixture(&f2);
	}
	
	void dominos_t::step(settings_t *settings){
		base_sim_t::step(settings);
		for(unsigned int i=0;i<no_of_metals;i++){
			b2Vec2 force(0,0),tot_force(0,0);
			b2Vec2 r1=metals[i]->GetWorldCenter();
			for(unsigned int j=0;j<no_of_magnets;j++){
				b2Vec2 r2=magnets[j]->GetWorldCenter();
				if(forceon[j]==-1){
					force+=r2;
					force-=r1;
				}
				else{
					force+=r1;
					force-=r2;
				}
				float r=pow(pow((r1.x-r2.x),2)+pow((r1.y-r2.y),2),0.5);
				force*=k/pow(r,3);
				tot_force+=force;
				force.x=0;
				force.y=0;

			}
			metals[i]->ApplyForceToCenter(tot_force,true);
		}
	}	
	
	void dominos_t::keyboard_up(unsigned char key){
		switch(key){
			case 'Q':	{
									for(unsigned int i=0;i<no_of_magnets;i++){
										forceon[i]=-1;
									}
									break;
								}
			case 'T': {
									for(unsigned int i=0;i<no_of_magnets;i++){
										forceon[i]=(-forceon[i]);
									}
									break;
								}
			default: base_sim_t::keyboard(key);
							 break;
		}
	}
	void dominos_t::mouse_down(const b2Vec2& p){
		for(unsigned int i=0;i<no_of_magnets;i++){
			b2Vec2 r1=magnets[i]->GetWorldCenter();
			if(sqrt(pow(r1.x-p.x,2)+pow(r1.y-p.y,2))<3){
				forceon[i]=-forceon[i];
			}
		}
		base_sim_t::mouse_down(p);
	}
	sim_t *sim = new sim_t("Dominos", dominos_t::create);
}
