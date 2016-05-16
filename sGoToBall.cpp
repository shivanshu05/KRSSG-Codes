#include "skillSet.h"
#include <ssl_common/grSimComm.h>
#include <ssl_common/config.h>
#include <navigation/planners.h>
#include <navigation/controllers/waypoint.h>
#include <cstdio>
#include <vector>
#include <navigation/planners.h>
#include <navigation/controllers/waypoint.h>
// #include "logger.h"
// #include "timer.h"
#include <ssl_common/config.h>
#include <ssl_common/grSimComm.h>

// static Util::Timer  timer;
// extern Util::Logger logger;

//Add comm.addLine & comm.addCircle
#define POINTPREDICTIONFACTOR 2

using namespace std;

namespace Strategy
{
  gr_Robot_Command SkillSet::goToBall(const SParam &param, const BeliefState &state, int botID)
  {
    using Navigation::obstacle;
#if 1
    vector<obstacle> obs;
    for(int i = 0; i < state.homeDetected.size(); ++i)
    {
      if (state.homeDetected[i]) {
        obstacle o;
        o.x = state.homePos[i].x;
        o.y = state.homePos[i].y;
        o.radius = 2 * BOT_RADIUS;
        obs.push_back(o);
      }
    }

    for(int i = 0; i < state.awayDetected.size(); ++i)
    {
      if (state.awayDetected[i]) {
        obstacle o;
        o.x = state.awayPos[i].x;
        o.y = state.awayPos[i].y;
        o.radius = 2 * BOT_RADIUS;
        obs.push_back(o);
      }

    }
    Vector2D<int> ballfinalpos;
    ballfinalpos.x = state.ballPos.x + (state.ballVel.x / POINTPREDICTIONFACTOR);
    ballfinalpos.y = state.ballPos.y + (state.ballVel.y / POINTPREDICTIONFACTOR);
    Vector2D<int> point, nextWP, nextNWP;
    Navigation::MergeSCurve pathPlanner;
    Vector2D<int> botPos(state.homePos[botID].x, state.homePos[botID].y);

    pathPlanner.plan(botPos,
                      ballfinalpos,
                      &nextWP,
                      &nextNWP,
                      &obs[0],
                      obs.size(),
                      botID,
                      true);


#else
    
#endif

#if 1
    float motionAngle = Vector2D<int>::angle(nextWP, botPos);

    float finalSlope;   // final slope the current bot motion should aim for!
    if(nextNWP.valid())
      finalSlope = Vector2D<int>::angle(nextNWP, nextWP);
    else
      finalSlope = Vector2D<int>::angle(nextWP, botPos);

    float turnAngleLeft = normalizeAngle(finalSlope - state.homePos[botID].theta); // Angle left to turn

    float omega = turnAngleLeft * MAX_BOT_OMEGA / (2 * PI); // Speedup turn
    if(omega < MIN_BOT_OMEGA && omega > -MIN_BOT_OMEGA)
    {
      if(omega < 0) omega = -((2.0f*MIN_BOT_OMEGA+5.0f*MAX_BOT_OMEGA)/7.0f);
      else omega = ((2.0*MIN_BOT_OMEGA+5.0*MAX_BOT_OMEGA)/7.0f);
      //MYSELF: if(omega < 0) omega = -MIN_BOT_OMEGA;
      //MYSELF: else omega = MIN_BOT_OMEGA;
    }

    float dist = Vector2D<int>::dist(nextWP, botPos);  // Distance of next waypoint from the bot
    float theta =  motionAngle - state.homePos[botID].theta;               // Angle of dest with respect to bot's frame of reference

    //MYSELF: float profileFactor = (dist * 4 / MAX_FIELD_DIST) * MAX_BOT_SPEED;
    float profileFactor = (dist * 2 / MAX_FIELD_DIST) * MAX_BOT_SPEED;

    //MYSELF: if(profileFactor < MIN_BOT_SPEED && dist > BOT_BALL_THRESH)
    if(profileFactor < MIN_BOT_SPEED & dist > BOT_BALL_THRESH)
      profileFactor = MIN_BOT_SPEED;
    
    else if(profileFactor > MAX_BOT_SPEED)
      profileFactor = MAX_BOT_SPEED;
#ifdef GR_SIM_COMM
    if(dist < BOT_BALL_THRESH * 5)
      profileFactor = 0.5f * (dist / MAX_FIELD_DIST * MAX_BOT_SPEED);
      //MYSELF: profileFactor = dist / MAX_FIELD_DIST * MAX_BOT_SPEED;
#endif
    if(param.GoToBallP.intercept == false)
    {
      if (dist < DRIBBLER_BALL_THRESH)
      {
        if(dist < BOT_BALL_THRESH)
        {
          if((turnAngleLeft) > -DRIBBLER_BALL_ANGLE_RANGE  && (turnAngleLeft) < DRIBBLER_BALL_ANGLE_RANGE)
          {
//            printf("not moving\n");
            return getRobotCommandMessage(botID, 0, 0, 0, 0, true);
          }
          else
          {
//            printf("turning\n");
            return getRobotCommandMessage(botID, 0, 0, omega, 0, true);
          }
        }
        else
        {
          // Make the dribbler on
//				if ((turnAngleLeft > -DRIBBLER_BALL_ANGLE_RANGE) && (turnAngleLeft < DRIBBLER_BALL_ANGLE_RANGE))
//				{
//          printf("not moving");
//					return getRobotCommandMessage(botID, 0, 0, 0, 0, true);
//				}
//				else
          {
//            printf("Moving: %f, %f\n", profileFactor, omega);
            return getRobotCommandMessage(botID, profileFactor * sin(-theta), profileFactor * cos(-theta), omega, 0, true);
          }
        }
      }
      else
      {
//        printf("Moving: %f, %f\n", profileFactor, omega);
        return getRobotCommandMessage(botID, profileFactor * sin(-theta), profileFactor * cos(-theta), omega, 0, false);
      }
    }
    else
    {
      if(dist > BOT_BALL_THRESH)
      {
//        printf("Moving: %f, %f\n", profileFactor, omega);
        return getRobotCommandMessage(botID, profileFactor * sin(-theta), profileFactor * cos(-theta), 0, 0, false);
      }
      else
      {
//        printf("Moving: %f, %f\n", profileFactor, omega);
        return getRobotCommandMessage(botID, 0, 0, 0, 0, true);
      }

    }

#else
    return getRobotCommandMessage(botID, 0, 0, 0, 0, false);
#endif
  } // goToBall
}
