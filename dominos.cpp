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
namespace cs251
{
	/**  The is the constructor 
	 * This is the documentation block for the constructor.
	 */ 
	
	dominos_t::dominos_t(){
		k=100000;
		for(int i=0;i<4;i++){
			forceon[i]=true;
		}
		b2BodyDef boundary;
		boundary.type=b2_staticBody;
		b2Body *bound;
		bound=m_world->CreateBody(&boundary);
		b2EdgeShape lines;
		b2Vec2 arr[]={b2Vec2(-30,0),b2Vec2(30,0),b2Vec2(30,40),b2Vec2(-30,40)};
		for(int i=0;i<4;i++){
			lines.Set(arr[i],arr[(i+1)%4]);
			bound->CreateFixture(&lines,0.0f);
		}
		//m_world->SetGravity(b2Vec2(0,0));
		b2BodyDef body;
		body.type=b2_staticBody;
		b2CircleShape circle;
		circle.m_radius=5;
		circle.m_p.Set(0,0);
		b2FixtureDef f1;
		f1.shape=&circle;
		f1.restitution=1;
		f1.friction=0;
		for(int i=0,j=-20;i<2;i++,j+=40){
			body.position.Set(j,20);
			b_magnetic[i]=m_world->CreateBody(&body);
			b_magnetic[i]->CreateFixture(&f1);
		}
		body.type=b2_dynamicBody;
		circle.m_radius=2;
		f1.density=1;
		for(int i=0,j=10,k=-10;i<2;i++,j+=20,k+=30){
			body.position.Set(0,j);
			b_metallic[i]=m_world->CreateBody(&body);
			b_metallic[i]->CreateFixture(&f1);
			b_metallic[i]->SetLinearVelocity(b2Vec2(0,0));
		}     
	}
	void dominos_t::step(settings_t *settings){
		base_sim_t::step(settings);
		for(int i=0;i<2;i++){
			b2Vec2 force(0,0),tot_force(0,0);
			b2Vec2 r1=b_metallic[i]->GetWorldCenter();
			for(int j=0;j<2;j++){
				b2Vec2 r2=b_magnetic[j]->GetWorldCenter();
				if(forceon[2*i+j]){
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
				//cout<<tot_force.x<<" "<<tot_force.y<<endl;
			}
			b_metallic[i]->ApplyForceToCenter(tot_force,true);
		}
		
	}
	void dominos_t::keyboard(unsigned char key){
		switch(key){
			case 'Q':	{
									for(int i=0;i<4;i++){
										forceon[i]=true;
									}
									break;
								}
			case 'T': {
									for(int i=0;i<4;i++){
										forceon[i]=(!forceon[i]);
									}
									break;
								}
			default: base_sim_t::keyboard(key);
							 break;
		}
	}
	void dominos_t::mouse_down(const b2Vec2& p){
		b2Vec2 r1=b_magnetic[0]->GetWorldCenter(),r2=b_magnetic[1]->GetWorldCenter();
		if(sqrt(pow(r1.x-p.x,2)+pow(r1.y-p.y,2))<5){
			for(int i=0;i<3;i+=2)
				forceon[i]=(!forceon[i]);
		}
		else if(sqrt(pow(r2.x-p.x,2)+pow(r2.y-p.y,2))<5){
			for(int i=1;i<4;i+=2){
				forceon[i]=(!forceon[i]);
			}
		}
		else{
			base_sim_t::mouse_down(p);
		}
	}
	sim_t *sim = new sim_t("Magnetic Materials Demo", dominos_t::create);
}
