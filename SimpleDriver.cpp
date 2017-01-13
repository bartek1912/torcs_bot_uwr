/***************************************************************************
 
    file                 : SimpleDriver.cpp
    copyright            : (C) 2007 Daniele Loiacono
 
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/
#include "SimpleDriver.h"
const double mnoznik_kier = 8.5711/2;



double last_dirs[100];
vector<double> corners;
vector<double> end_cor;
vector<int> minimal_speed, out_speed;
int ostatnio_karany = -1, breaking_for = 0;
int STEP = 4;
CarControl SimpleDriver::wDrive(CarState cs)
{
  static bool inside_corner = false;
  static int next_corner = 0;
  static double last_dir = 0;
  static bool is_wrong_dir = false;
  double dir;
  if(abs(cs.getTrackPos()) > 1)
  {
    int closest = next_corner+1;//breaking_for;
    breaking_for = -1;
    {
      if(inside_corner)
      {
        if(ostatnio_karany != closest-2 && closest != -1)
        {
          cerr<<"KARZE in "<<closest-2<<" with "<<STEP+1<<" from "<<minimal_speed[closest-2]<<"                                           \n";
          minimal_speed[closest-2] -= 4*STEP + 1;
          out_speed[closest-2] -= 2*STEP + 1;
          ostatnio_karany = closest-2;        
        }
      }
      else
      {
        if(ostatnio_karany != closest-1 && closest != -1)
        {
          cerr<<"KARZE out "<<closest-1<<" with "<<STEP+1<<" from "<<minimal_speed[closest-1]<<"                                           \n";
          out_speed[closest-1] -= 4*STEP+1;
          ostatnio_karany = closest-1;
        }
      }
    }
    if(abs(cs.getSpeedY()) > 5)
      return CarControl(0, 1, 0, 0, 0);
    else if(((cs.getAngle() > 0 && cs.getTrackPos() < 0 )
              || (cs.getAngle() < 0 && cs.getTrackPos() > 0))&& cs.getSpeedX() < 10)
      return CarControl(0.7, 0, -1, (cs.getTrackPos() < 0 ? -1 : 1), 0.00);
    else
      return CarControl(1, 0, 1, (cs.getTrackPos() < 0 ? (cs.getAngle() > -PI/4 ? 0.7 : -0.5) : (cs.getAngle() < PI/4 ? -0.7 : 0.5)), 0.00);
  }
  /*else if(cs.getTrackPos() > 1)
  {
    int closest = breaking_for;
    breaking_for = -1;
    if(ostatnio_karany != closest && closest != -1)
    {
      cerr<<"KARZE "<<closest<<" WITH "<<STEP+1<<"\n";
      minimal_speed[closest] -= STEP+1;
      out_speed[closest] -= STEP + 1;
      ostatnio_karany = closest;
    }
    if(abs(cs.getSpeedY()) > 5)
      return CarControl(0, 1, 0, 0, 0);
    else if(cs.getAngle() < 0 && cs.getSpeedX() < 10)
      return CarControl(0.7, 0, -1, 1, 0.00);
    else
      return CarControl(1, 0, 1, (cs.getAngle() < PI/4 ? -0.7 : 0.5), 0.00);
  }*/
  else if(abs(cs.getAngle()) > 3*PI/4 - PI/2 * is_wrong_dir && cs.getGear() > 0
          || abs(cs.getAngle()) > PI/4 && cs.getSpeedX() < 20)
  {
    is_wrong_dir = true;
    return CarControl(0.8, 0, 1, 0.7*(cs.getAngle() < 0? -1: 1)*(cs.getSpeedX()<0?-1:1), (abs(cs.getSpeedX())<15? 0.8 : 0.00));
  }
  else if(abs(cs.getSpeedX()) < 15 && cs.getDistFromStart() > 100 && (cs.getLastLapTime() >  0 || cs.getCurLapTime() > 2))
  {
    return CarControl(0.8, 0, -1, 0, 0);
  }
  else
  {
   
    is_wrong_dir = false;
    int id_mx = TRACK_SENSORS_NUM/2;
    for(int i = 0; i < TRACK_SENSORS_NUM; i++)
      if(cs.getTrack(i) > cs.getTrack(id_mx))
          id_mx = i;

    if(min(cs.getTrack(id_mx-1), min(cs.getTrack(id_mx), cs.getTrack(id_mx+1))) < 100)
      dir = -(id_mx - TRACK_SENSORS_NUM/2)/static_cast<double>(TRACK_SENSORS_NUM) * mnoznik_kier;
    else
      dir = 0;
    if(-1 < cs.getTrackPos()  && cs.getTrackPos() < -0.9 && dir < -0.05)
      dir += 0.1;
    if(1 > cs.getTrackPos()  && cs.getTrackPos() > 0.9 && dir > 0.05)
      dir -= 0.1;
    dir = (3*last_dir + dir)/4;
    last_dir = dir;
    double acc = 1, br = 0;
    if(cs.getLastLapTime() <= 0)
    {
      //100 - 30
      //200 - 60
      //300 - 270
      int bezp = 45;
      if((cs.getTrack(id_mx) < 150 && cs.getSpeedX() > 240 - bezp)
        || (cs.getTrack(id_mx) < 100 && cs.getSpeedX() > 220 - bezp)
        || (cs.getTrack(id_mx) < 60 && cs.getSpeedX() > 180 - bezp)
        || (cs.getTrack(id_mx) < 50 && cs.getSpeedX() > 160 - bezp)
        || (cs.getTrack(id_mx) < 40 && cs.getSpeedX() > 120 - bezp)
        || (cs.getTrack(id_mx) < 30 && cs.getSpeedX() > 100 - bezp)
        || (cs.getTrack(id_mx) < 20 && cs.getSpeedX() > 70 - bezp))
      {
        br = 1;
        acc = 0;
      }
      static int act_cor = 0, act_dir = 0;
      static bool corner_start = false;
      double sum = 0;
      for(auto d: last_dirs)
        sum += d;
      if(cs.getSpeedZ() > 20)
        cerr<<"Upward "<<cs.getSpeedZ()<<"\n";
      if((abs(sum) > 0.3 || cs.getSpeedZ() > 20) && !corner_start && cs.getSpeedX() > 20)
      {
        cerr<<"Nowy zakret "<<sum<<" ["<<minimal_speed.size()<<"] s:"<<cs.getSpeedX()<<" <= "<<( cs.getSpeedX()/3.6*2)<<" ";
        corner_start = true;
        corners.push_back(cs.getDistFromStart() - cs.getSpeedX()/3.6*2);
        minimal_speed.push_back(cs.getSpeedX() * (cs.getSpeedZ() > 20? 0.3: 1));
      }
      else if((abs(sum) < 0.3 && cs.getSpeedZ() < 15 ) && corner_start)
      {
        cerr<<" out "<<cs.getSpeedX()<<"\n";
        //corners.rbegin() = min(corners.rbegin(), cs.getDistFromStart());
        end_cor.push_back(cs.getDistFromStart() - cs.getSpeedX()/3.6*2);
        out_speed.push_back(cs.getSpeedX());
        corner_start = false;
      }
      if(abs(dir) > 0.2)
        acc /= 2;
      last_dirs[(act_dir++)%100] = dir;
      return CarControl(acc, br, cs.getGear(), dir, 0.00);
    }
    else
    {
      if(*corners.rbegin() != 1e9)
      {
        if(corners.size() > end_cor.size())
        {
          end_cor.push_back(*corners.rbegin() + 400);
          double mn = *minimal_speed.rbegin() * 0.75;
          mn = min(static_cast<double>(cs.getSpeedX()), mn);
          out_speed.push_back(mn);
        }
        corners.push_back(1e9);
        minimal_speed.push_back(100);
        end_cor.push_back(1e9);
        out_speed.push_back(100);
      }
      double delta = cs.getSpeedX() - (inside_corner ? out_speed[next_corner-1] : minimal_speed[next_corner]);
      double dist = (inside_corner ? end_cor[next_corner-1] : corners[next_corner]) - cs.getDistFromStart(), require_space = 0;
      if(delta > 0)
      {
        double require_time = delta / 100.00 * 2.16 ;
        require_space = (cs.getSpeedX() - delta)/3.6/2 * require_time * (inside_corner? 2 : 1);
        if(dist <= require_space || inside_corner)
        {
          cerr<<"BREAKING ";
          breaking_for = next_corner;
          acc = 0;
          br = (inside_corner? 1: 0);
        }
      }
      cerr<<"do zakretu["<<(inside_corner?next_corner-1:next_corner)<<"] "<<setw(6)<<fixed<<setprecision(1)<<dist<<" sp: "<<fixed<<
              (inside_corner?out_speed[next_corner-1]:minimal_speed[next_corner])<<"["<<require_space<<"] INC "<<inside_corner<<"                               \r";
      if(corners[next_corner] <= cs.getDistFromStart() && next_corner + 1 != corners.size())
      {
        inside_corner = 1;
        if(abs(cs.getSpeedX() - minimal_speed[next_corner]) > 10)
          minimal_speed[next_corner] -= STEP;
        next_corner = (next_corner+1)%corners.size();
        minimal_speed[next_corner] += STEP;
      }
      if(inside_corner)
        if(end_cor[next_corner-1] <= cs.getDistFromStart() && next_corner + 1 != corners.size())
        {
          if(abs(cs.getTrackPos()) < 0.9)
            out_speed[next_corner-1] += STEP;
          inside_corner = false;
        }

      if(cs.getDistFromStart() < 10)
      {
        if(next_corner)
        {
          //STEP *= 2.0/4;
          minimal_speed[0] += STEP/2;
        }
        inside_corner = 0;
        next_corner = 0;
        ostatnio_karany = -1;
        static ofstream plik("output", fstream::out | fstream::app);
        for(int i = 0; i < corners.size(); i++)
        {
          //cerr<<fixed<<minimal_speed[i]<<"("<<fixed<<corners[i]<<") -> ";
          plik<<fixed<<minimal_speed[i]<<"("<<fixed<<corners[i]<<") -> ";
        }
        plik<<"\n------------------------------"<<endl;
      }
      if(inside_corner)
        if(out_speed[next_corner-1] + 10 < cs.getSpeedX())
        acc = 0.3;
      else if(abs(dir) > 0.1)
        acc /= 3;
    }
    filterABS(cs, br);
    return CarControl(acc, br, cs.getGear(), dir, 0.00);
  }
} 


float SimpleDriver::filterABS(CarState &cs,float brake) const 
{
  float speed = cs.getSpeedX() / 3.6;
  
    if (speed < absMinSpeed)
        return brake;
    
    float slip = 0.0f;
    
    for (int i = 0; i < 4; i++){
        slip += cs.getWheelSpinVel(i) * wheelRadius[i];}
        
    slip = speed - slip/4.0f;
    
    if (slip > absSlip){
        brake = brake - (slip - absSlip)/absRange; }
    
    if (brake<0)
      return 0;
    else
      return brake;
}

void
SimpleDriver::onShutdown()
{
   // cout << "Bye bye!" << endl;
}

void
SimpleDriver::onRestart() 
{
}



/* ABS Filter Constants */
const float SimpleDriver::wheelRadius[4]={0.3179,0.3179,0.3276,0.3276};
const float SimpleDriver::absSlip=2.0;
const float SimpleDriver::absRange=3.0;
const float SimpleDriver::absMinSpeed=3.0;