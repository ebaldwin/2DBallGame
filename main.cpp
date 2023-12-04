/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/cpplite/CPPTemplate.cpp to edit this template
 */

#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <cstring>
#include <cstdint>
#include "raylib.h"
using namespace std;

//some global variables
extern float drag = 0.25;
extern float speedLimit = 100;
extern float pressImpulse = 4000;
extern float pressHoldAccel = 350;
extern float coeffOfRestitution = 1; //must be above 0 and logically less than 1, but more that 1 might be interesting...
extern float cR = coeffOfRestitution*.5 + 0.5;

class Ball
{
public:
  Vector2 pos;
  Vector2 vel = {0,0};
  Vector2 accel = {0,0};
  float mass = 1.0;
  float radius = 10;
  Color bColor;
  bool pocket = 0;
  Ball(float px, float py, float r, float m, Color bC, bool p)
  {
    pos = {px, py};
    bColor = bC;
    radius = r;
    mass = m;
    pocket = p;
  }
  void draw()
  {
    DrawCircleV(pos,radius,bColor);
  }
  void updatePos(float t)
  {
    vel.x += t*(accel.x -drag*vel.x);
    vel.y += t*(accel.y -drag*vel.y);
    pos.x += t*vel.x;
    pos.y += t*vel.y;
  }

  void wallBounce()
  {
    if(!pocket)
    {
      if((pos.x > (1220-radius)) && (vel.x >= 0))
      {
        vel.x *= -0.7;
      }
      if((pos.y > (620-radius)) && (vel.y >= 0))
      {
        vel.y *= -0.7;
      }
      if((pos.x < (20+radius)) && (vel.x <= 0))
      {
        vel.x *= -0.7;
      }
      if((pos.y < (20+radius)) && (vel.y <= 0))
      {
        vel.y *= -0.7;
      }
    }
  }
  float ke(){return 0.5*mass*(pow(vel.x,2)+pow(vel.y,2));}
  float momX(){return mass*vel.x;}
  float momY(){return mass*vel.y;}
};

class Wall
{
  public:
  Vector2 point1;
  Vector2 point2;
  Color wColor;
  Wall(float p1x, float p1y, float p2x, float p2y, Color wC)
  {
    point1 = {p1x,p1y};
    point2 = {p2x,p2y};
    wColor = wC;
  }
  void draw()
  {
    DrawLineV(point1,point2,wColor);
  }
};

float distBtwBalls(Ball *b1, Ball *b2)
{
  return sqrt((pow(b1->pos.x - b2->pos.x,2))+(pow(b1->pos.y - b2->pos.y,2))) - b1->radius - b2->radius;
}

float distBtwBallAndWall(Ball *b, Wall *w)
{
  return (abs((w->point2.x-w->point1.x)*(w->point1.y-b->pos.y)-(w->point1.x-b->pos.x)*(w->point2.y-w->point1.y))/sqrt(pow(w->point2.x-w->point1.x,2)+pow(w->point2.y-w->point1.y,2))) - b->radius;
}

float calcCollisionTime(Ball *b1, Ball *b2)
{
  //relative distance and velocity vectors from ball1 to ball2
  Vector2 relDist = {b2->pos.x - b1->pos.x, -1*(b2->pos.y - b1->pos.y)};
  Vector2 relVel = {b2->vel.x - b1->vel.x, -1*(b2->vel.y - b1->vel.y)};

  //check if distance is decreasing
  float drDotUr = relDist.x*relVel.x + relDist.y*relVel.y;
  if (drDotUr >= 0){return 7e9;}//positive dot product means balls are diverging, return large value...
  //cout<<"In calColT and drDotUr is: "<<drDotUr<<","<<relDist.x<<","<<relVel.x<<endl;

  //check if collision will occur
  float magRelDist = sqrt(relDist.x*relDist.x + relDist.y*relDist.y);
  float magRelVel = sqrt(relVel.x*relVel.x + relVel.y*relVel.y);
  float theta = 3.141596265 - acos(drDotUr / (magRelDist*magRelVel));
  float cdr = magRelDist*sin(theta);//closest rel distance if trajectories do not change
  if (cdr >= (b1->radius + b2->radius)){return 7e9;}//return large positive value if no collision pending

  //calculate collision time
  float dtcdr = magRelDist*cos(theta);//distance to cdr
  float dtcdrB = sqrt(pow(b1->radius + b2->radius,2)-pow(cdr,2));
  float dtcdrA = dtcdr - dtcdrB;
  //cout<<"dtcdr = "<<dtcdr<<", colTime = "<<dtcdrA/magRelVel<<endl;
  return dtcdrA/magRelVel;
}

void resolveBallCollision(Ball *b1, Ball *b2)
{
  if(b1->pocket)
  {
    b2->pos.x = 7e9;
    b2->vel.x = 0;
    //b2->pocket = 1; //to prevent wall rebound forces from messing with it
  }
  if(b2->pocket)
  {
    b1->pos.x = 7e9;
    b1->vel.x = 0;
    //b1->pocket = 1; //to make sure it stays gone forever
  }
  else
  {
    //impulse direction unit vector
    Vector2 imp = {(b1->pos.x-b2->pos.x)/sqrt(pow(b1->pos.x-b2->pos.x,2)+pow(b1->pos.y-b2->pos.y,2)),
    (b1->pos.y - b2->pos.y)/sqrt(pow(b1->pos.x - b2->pos.x,2)+pow(b1->pos.y - b2->pos.y,2))};
    //quadratic formula solve to get the impulse magnitude
    float impMag = -2*(imp.x*(b1->vel.x-b2->vel.x)+imp.y*(b1->vel.y-b2->vel.y))
    /
    (((pow(imp.x,2)+pow(imp.y,2))/(b1->mass))+((pow(imp.x,2)+pow(imp.y,2))/(b2->mass)));
    //update ball velocities
  /*  cout<<"imp mag is: "<<impMag<<endl;
    cout<<"b1 KE before collision is: "<<b1->ke()<<" and mom is: "<<b1->momX()<<", "<<b1->momY()<<endl;
    cout<<"b2 KE before collision is: "<<b2->ke()<<" and mom is: "<<b2->momX()<<", "<<b2->momY()<<endl;
    cout<<"total KE is: "<<b1->ke()+b2->ke()<<" and mom is: "<<b1->momX()+b2->momX()<<", "<<b1->momY()+b2->momY()<<endl;*/
    b1->vel.x += impMag*cR*imp.x*(1/b1->mass);
    b1->vel.y += impMag*cR*imp.y*(1/b1->mass);
    b2->vel.x -= impMag*cR*imp.x*(1/b2->mass);
    b2->vel.y -= impMag*cR*imp.y*(1/b2->mass);

  /*  cout<<"b1 KE after collision is: "<<b1->ke()<<" and mom is: "<<b1->momX()<<", "<<b1->momY()<<endl;
    cout<<"b2 KE after collision is: "<<b2->ke()<<" and mom is: "<<b2->momX()<<", "<<b2->momY()<<endl;
    cout<<"total KE is: "<<b1->ke()+b2->ke()<<" and mom is: "<<b1->momX()+b2->momX()<<", "<<b1->momY()+b2->momY()<<endl;*/
  }
}

int main()
{
  //generate starting environment - this could be done in a variety of ways
  //hard code it for now...
  //player1
  Ball ballP1 = Ball(100,250,13.5,1,RED,0);
  //player2
  Ball ballP2 = Ball(100,450,13.5,1,DARKBLUE,0);
  //pool balls
  Ball ball1 = Ball(920,320,13.5,1,WHITE,0);
  Ball ball2 = Ball(944.25,334,13.5,1,GRAY,0);
  Ball ball3 = Ball(944.25,306,13.5,1,WHITE,0);
  Ball ball4 = Ball(968.5,348,13.5,1,WHITE,0);
  Ball ball8 = Ball(968.5,320,13.5,1,BLACK,0);
  Ball ball6 = Ball(968.5,292,13.5,1,GRAY,0);
  Ball ball7 = Ball(992.75,362,13.5,1,GRAY,0);
  Ball ball5 = Ball(992.75,334,13.5,1,WHITE,0);
  Ball ball9 = Ball(992.75,306,13.5,1,GRAY,0);
  Ball ball10 = Ball(992.75,278,13.5,1,GRAY,0);
  Ball ball11 = Ball(1016.99,376,13.5,1,WHITE,0);
  Ball ball12 = Ball(1016.99,348,13.5,1,WHITE,0);
  Ball ball13 = Ball(1016.99,320,13.5,1,GRAY,0);
  Ball ball14 = Ball(1016.99,292,13.5,1,WHITE,0);
  Ball ball15 = Ball(1016.99,264,13.5,1,GRAY,0);

  //pockets modeled by balls b/c the collision math is already worked out...
  Ball pocket1 = Ball(20,20,137,7e9,DARKBROWN,1);
  Ball pocket2 = Ball(620,20,137,7e9,DARKBROWN,1);
  Ball pocket3 = Ball(1220,20,137,7e9,DARKBROWN,1);
  Ball pocket4 = Ball(20,620,137,7e9,DARKBROWN,1);
  Ball pocket5 = Ball(620,620,137,7e9,DARKBROWN,1);
  Ball pocket6 = Ball(1220,620,137,7e9,DARKBROWN,1);

  //walls
  Wall wall1(20,20,20,620,BROWN);
  Wall wall2(20,620,1220,620,BROWN);
  Wall wall3(1220,20,1220,620,BROWN);
  Wall wall4(20,20,1220,20,BROWN);
  //generate vector of balls and walls
  vector<Ball*> ballPtrVect;
  ballPtrVect.push_back(&ballP1);
  ballPtrVect.push_back(&ballP2);
  ballPtrVect.push_back(&ball1);
  ballPtrVect.push_back(&ball2);
  ballPtrVect.push_back(&ball3);
  ballPtrVect.push_back(&ball4);
  ballPtrVect.push_back(&ball5);
  ballPtrVect.push_back(&ball6);
  ballPtrVect.push_back(&ball7);
  ballPtrVect.push_back(&ball8);
  ballPtrVect.push_back(&ball9);
  ballPtrVect.push_back(&ball10);
  ballPtrVect.push_back(&ball11);
  ballPtrVect.push_back(&ball12);
  ballPtrVect.push_back(&ball13);
  ballPtrVect.push_back(&ball14);
  ballPtrVect.push_back(&ball15);
  ballPtrVect.push_back(&pocket1);
  ballPtrVect.push_back(&pocket2);
  ballPtrVect.push_back(&pocket3);
  ballPtrVect.push_back(&pocket4);
  ballPtrVect.push_back(&pocket5);
  ballPtrVect.push_back(&pocket6);
  vector<Wall*> wallPtrVect;
  wallPtrVect.push_back(&wall1);
  wallPtrVect.push_back(&wall2);
  wallPtrVect.push_back(&wall3);
  wallPtrVect.push_back(&wall4);

  int nBalls = ballPtrVect.size();  //number of balls in the program
  int nWalls = wallPtrVect.size(); //number of walls in the program (just four for now)

  //iterate over balls checking for spatial conflicts in ICs
  for (int i0 = 0; i0 < nBalls; i0++){
    for (int i1 = i0+1; i1 < nBalls; i1++){
      if(ballPtrVect[i0]->pocket || ballPtrVect[i1]->pocket){continue;}
      if (distBtwBalls(ballPtrVect[i0],ballPtrVect[i1]) < 0.0)
      {
        cout << "There is a conflict in the initial conditions! Check balls located at:" << endl;
        cout << "x=" << ballPtrVect[i0]->pos.x << " y=" << ballPtrVect[i0]->pos.y <<
          " and x=" << ballPtrVect[i1]->pos.x << " y=" << ballPtrVect[i1]->pos.y << endl << endl;
        cout << "program exiting..." << endl;
        return 0;
      }
    }
    cout << endl;
  }
  //then check for conflicts with the walls...
  //iterate over balls checking conflicts in ICs with walls
  for (int i0 = 0; i0 < nBalls; i0++){
    for (int i1 = 0; i1 < nWalls; i1++){
      if(ballPtrVect[i0]->pocket){continue;}
      float nDist = distBtwBallAndWall(ballPtrVect[i0],wallPtrVect[i1]);
      cout << "nDist ball to wall is: " << nDist << endl;
      if (nDist < 0.0)
      {
        cout << "There is a wall-ball conflict in the initial conditions! Check ball located at:" << endl;
        cout << "x=" << ballPtrVect[i0]->pos.x << " y=" << ballPtrVect[i0]->pos.y << endl << endl;
        cout << "program exiting..." << endl;
        return 0;
      }
    }
    cout << endl;
  }

  //Name ideas from the kids...
  //InitWindow(1000, 700, "Ball Knocker!!!");
  InitWindow(1240, 700, "Knock Balls!!!");
  SetWindowState(FLAG_VSYNC_HINT);

  float frameTime = (1/60.0);
  float dt;
  bool collisionFound;
  bool clearToAdvanceT;
  float soonestPendingCollisionTime;
  float pendingColTime;
  int collisionBall1;
  int collisionBall2;

  //const char* frameTimeMessage = nullptr;
  while(!WindowShouldClose())
  {
    dt = frameTime;

    //arrows cause acceleration on ball1
    ballP1.accel.x = 0;
    ballP1.accel.y = 0;
    if(IsKeyDown(KEY_UP)){ballP1.accel.y -= pressHoldAccel;}
    if(IsKeyDown(KEY_DOWN)){ballP1.accel.y += pressHoldAccel;}
    if(IsKeyDown(KEY_LEFT)){ballP1.accel.x -= pressHoldAccel;}
    if(IsKeyDown(KEY_RIGHT)){ballP1.accel.x += pressHoldAccel;}
    if(IsKeyPressed(KEY_UP)){ballP1.vel.y -= pressImpulse*frameTime;}
    if(IsKeyPressed(KEY_DOWN)){ballP1.vel.y += pressImpulse*frameTime;}
    if(IsKeyPressed(KEY_LEFT)){ballP1.vel.x -= pressImpulse*frameTime;}
    if(IsKeyPressed(KEY_RIGHT)){ballP1.vel.x += pressImpulse*frameTime;}

    //WASD for player 2
    ballP2.accel.x = 0;
    ballP2.accel.y = 0;
    if(IsKeyDown(KEY_W)){ballP2.accel.y -= pressHoldAccel;}
    if(IsKeyDown(KEY_S)){ballP2.accel.y += pressHoldAccel;}
    if(IsKeyDown(KEY_A)){ballP2.accel.x -= pressHoldAccel;}
    if(IsKeyDown(KEY_D)){ballP2.accel.x += pressHoldAccel;}
    if(IsKeyPressed(KEY_W)){ballP2.vel.y -= pressImpulse*frameTime;}
    if(IsKeyPressed(KEY_S)){ballP2.vel.y += pressImpulse*frameTime;}
    if(IsKeyPressed(KEY_A)){ballP2.vel.x -= pressImpulse*frameTime;}
    if(IsKeyPressed(KEY_D)){ballP2.vel.x += pressImpulse*frameTime;}

    clearToAdvanceT = 0;
    soonestPendingCollisionTime = 7e9;
    pendingColTime = 7e9;
    collisionBall1 = -1;
    collisionBall2 = -1;
    while(!clearToAdvanceT)
    {
      collisionFound = 0;
      //iterate over balls checking for collisions
      for (int i0 = 0; i0 < nBalls; i0++)
      {
        if(ballPtrVect[i0]->pos.x > 1e9){continue;}//balls with a large positive x have been hit into pockets and are out of play...
        for (int i1 = i0+1; i1 < nBalls; i1++)
        {
          if(ballPtrVect[i1]->pos.x > 1e9){continue;}//balls with a large positive x have been hit into pockets and are out of play...
          pendingColTime = calcCollisionTime(ballPtrVect[i0],ballPtrVect[i1]);
          if(pendingColTime<dt)//could cause a bug?
          {
            soonestPendingCollisionTime = pendingColTime;
            collisionBall1 = i0;
            collisionBall2 = i1;
            collisionFound = 1;
            clearToAdvanceT = 0;
          }
        }
      }
      //if collision imminent, advance time to collision and resolve collision
      if(collisionFound)
      {
        for(int i0=0;i0<nBalls;i0++)
        {
          ballPtrVect[i0]->updatePos(soonestPendingCollisionTime);
        }
        dt -= soonestPendingCollisionTime;
        //resolve collision
        resolveBallCollision(ballPtrVect[collisionBall1],ballPtrVect[collisionBall2]);
      }
      //else advance to next frame
      else
      {
        //update ball positions
        for(int i0=0;i0<nBalls;i0++)
        {
          ballPtrVect[i0]->updatePos(dt);
        }
        //bounce off walls
        for(int i0=0;i0<nBalls;i0++)
        {
          ballPtrVect[i0]->wallBounce();
        }
        clearToAdvanceT=1;
      }
    }

    //draw things
    BeginDrawing();
    ClearBackground(BEIGE);
    for(int i0=0;i0<nWalls;i0++)
    {
      wallPtrVect[i0]->draw();
    }
    for(int i0=0;i0<nBalls;i0++)
    {
      ballPtrVect[i0]->draw();
    }
    DrawFPS(20,682);
    EndDrawing();
  }
  CloseWindow();
return 0;
}

//my initial pseudo code
  //display initial state
  //float DT = 1/screenHz;
  //float currentTime = 0;
  //float nextDisplayTime = currentTime + DT
  //boolean paused = 1;
  //boolean collisionFound;
  //while not paused:
  //  while (currentTime < nextDisplayTime){
  //    dt = nextDisplayTime - currentTime
  //    collisionFound = 0;
  //    loop through objects and check if there will be any collisions
  //      if collision found
  //        collisionFound = 1;
  //        solve for collision time and store pointers to the objects with the closest pending colission
  //        dt = closestPendingCollisionTime;
  //    if collisionFound{
  //      solve for the collision, advanving positions and velocities and time+dt
  //      update positions and velocities of all remaining objects to time+dt}
  //    else{update positions and velocities of all objects to time+dt}
  //    currentTime += dt
  //  when real time == nextDisplayTime or: update display to show new currentTime state
  //  nextDisplayTime = currentTime + DT
