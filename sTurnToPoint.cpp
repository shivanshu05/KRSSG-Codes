#include "skillSet.h"
#include <ssl_common/grSimComm.h>
#include <ssl_common/config.h>
#include <navigation/planners.h>
#include <navigation/controllers/waypoint.h>
#include <cstdio>
#include <vector>

#include <ssl_common/geometry.hpp>
namespace Strategy
{
  gr_Robot_Command SkillSet::turnToPoint(const SParam &param, const BeliefState &state, int botID)
  {
    Vector2D<int> point(param.TurnToPointP.x, param.TurnToPointP.y);
    Vector2D<int> botPos(state.homePos[botID].x, state.homePos[botID].y);
    Vector2D<int> ballPos(state.ballPos.x, state.ballPos.y);

    float finalSlope = Vector2D<int>::angle(point, botPos);
    float turnAngleLeft = normalizeAngle(finalSlope - state.homePos[botID].theta); // Angle left to turn
    float omega = turnAngleLeft * param.TurnToPointP.max_omega / (2 * PI); // Speedup turn
    if(omega < MIN_BOT_OMEGA && omega > -MIN_BOT_OMEGA)
    {
      if(omega < 0) omega = -(2.0*MIN_BOT_OMEGA+5.0*MAX_BOT_OMEGA/7.0);
      else omega = (2.0*MIN_BOT_OMEGA+5.0*MAX_BOT_OMEGA/7.0);
      //MYSLEF: if(omega < 0) omega = -MIN_BOT_OMEGA;
      //MYSELF: else omega = MIN_BOT_OMEGA;
    }
    float v_x = omega*BOT_BALL_THRESH*1.5;
    // comm.addCircle(state->homePos[botID].x,  state->homePos[botID].y, 50);
    float dist = Vector2D<int>::dist(ballPos, botPos);
    if(dist < DRIBBLER_BALL_THRESH*1.2)
    {
      return getRobotCommandMessage(botID, v_x, 0, omega, 0, true);
    }
    else
    {
      return getRobotCommandMessage(botID, 0, 0, omega, 0, false);
    }
  }
}
