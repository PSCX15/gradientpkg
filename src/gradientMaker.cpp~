#include "ros/ros.h"
#include "gradientpkg/mesure.h"
#include "gradientpkg/gradient.h"
#include <vector>
#include <math.h>
using namespace std; 

class Mesure
{
  public:
     float x;
     float y;
     float m;
     Mesure(float x, float y,float m);
};

Mesure::Mesure(float x,float y,float m)
{
  this->x=x;
  this->y=y;
  this->m=m;
}

class Grad
{
  public:
     float px;
     float py;
     float gx;
     float gy;
     float theta;
     Grad(float x,float y);
};

Grad::Grad(float x, float y)
{ 
  this->gx=x;this->gy=y;
}

ros::Publisher pub;
float dis = 15.0;
int compteur;
vector<Grad> gradient;
vector<Mesure> data;

void donneesCallback(const gradientpkg::mesure::ConstPtr& msg){
  
  Mesure m(msg->x,msg->y,msg->a);
  data.push_back(m);
  
  
  Grad g(0,0);
  int i,nb=0;
  if(compteur>0)
  {   
    for(i=0;i<compteur;i++)
    {
      Mesure a=data.at(compteur), b=data.at(i);
      float r1 = a.x-b.x, r2 = a.y-b.y;      

      if(fabs(r1)>dis||fabs(r2)>dis) continue;
      else if(pow(r1,2)+pow(r2,2)>pow(dis,2)) continue;
      else 
      {
        g.gx+=(a.m-b.m)*r1/(pow(r1,2)+pow(r2,2));
        g.gy+=(a.m-b.m)*r2/(pow(r1,2)+pow(r2,2));
        nb++;
      }
    }
    g.px=data.at(compteur).x;
    g.py=data.at(compteur).y;
    if(nb>=200) g.theta=2;
    else g.theta=90-0.44*nb;
    gradient.push_back(g);
    
    gradientpkg::gradient msg;

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "/gradient";
    msg.px=g.px; msg.py=g.py;
    msg.gx=g.gx/sqrt(pow(g.gx,2)+pow(g.gy,2)); 
    msg.gy=g.gy/sqrt(pow(g.gx,2)+pow(g.gy,2));
    msg.theta=g.theta;
    pub.publish(msg);
  }
  compteur++;    
}



int main(int argc, char **argv)
{
  compteur=0;  
  ros::init(argc, argv, "gradient");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("donnees",1000,donneesCallback);
  pub = n.advertise<gradientpkg::gradient>("ggradient",1000);
  ros::spin();

  return 0;
}
