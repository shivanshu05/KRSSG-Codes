#include <list>
#include "tKickToGoal1.hpp"
#include "skills/skillSet.h"
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include <iostream>
#include <stdio.h>
#include <ssl_common/geometry.hpp>
#include <skills/skillSet.h>


namespace Strategy
{
  TKickToGoal1::TKickToGoal1(int botID) : Tactic( botID) {
    goal = Vector2D<int>(HALF_FIELD_MAXX, 0);
  } // TTStop
    
  TKickToGoal::~TKickToGoal()
  { } // ~TTStop
    
  bool TKickToGoal1::isCompleted(const BeliefState &bs) const {
    return false;
  }
  
  inline bool TKickToGoal1::isActiveTactic(void) const {
    return true;
  }

  int TKickToGoal1::chooseBestBot(const BeliefState &state, std::list<int>& freeBots, const Param& tParam, int prevID) const {
    int minv = *(freeBots.begin());
    float mindis = -1.0f;
    for (std::list<int>::const_iterator it = freeBots.begin(); it != freeBots.end(); ++it) {
      
      Vector2D<int> homePos(state.homePos[*it].x, state.homePos[*it].y);
      Vector2D<int> ballPos(state.ballPos.x, state.ballPos.y);

      float dis_from_ball = (homePos - ballPos).absSq();
      if (mindis < 0) {
        mindis = dis_from_ball;
        minv = *it;
      }
      else if(dis_from_ball < mindis) {
          mindis = dis_from_ball;
          minv = *it;
        }
      }
    return minv;
  } 

  gr_Robot_Command TKickToGoal1::execute(const BeliefState &state, const Tactic::Param& tParam) {
    Strategy::SkillSet::SkillID sID;
    Strategy::SkillSet::SParam sparam;

    printf("AAA\n");
    Vector2D<int> botPos(state.homePos[botID].x, state.homePos[botID].y);
    Vector2D<int> ballPos(state.ballPos.x, state.ballPos.y);
    float dist = Vector2D<int>::dist(ballPos, botPos);
    float angleWithBall = normalizeAngle(state.homePos[botID].theta - Vector2D<int>::angle(ballPos, botPos));

    if((dist > BOT_BALL_THRESH * 3.0f || fabs(angleWithBall) > DRIBBLER_BALL_ANGLE_RANGE)) {
      printf("going to ball\n");
      sID = SkillSet::GoToBall; 
      return SkillSet::instance()->executeSkill(sID, sParam, state, botID);
    }
  }

  Tactic::Param TKickToGoal1::paramFromJSON(string json) {
    using namespace rapidjson;
    Tactic::Param tParam;
      
    return tParam;
  }

  string TKickToGoal1::paramToJSON(Tactic::Param tParam) {
    return string("");
  }
    
} // namespace Strategy

