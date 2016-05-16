#include "skillSet.h"
#include <ssl_common/grSimComm.h>
#include <ssl_common/config.h>
#include <navigation/planners.h>
#include <navigation/controllers/waypoint.h>

using namespace std;
namespace Strategy
{
  gr_Robot_Command SkillSet::goToPoint(const SParam &param, const BeliefState &state, int botID)
  {
    using Navigation::obstacle;
    Vector2D<int> dpoint;
    dpoint.x = param.GoToPointP.x;
    dpoint.y = param.GoToPointP.y;
//    printf("point= %d,%d\n",dpoint.x,dpoint.y);
    vector<obstacle> obs;
    for(int i = 0; i < state.homeDetected.size(); ++i)
    {
      if (state.homeDetected[i]) {
        obstacle o;
        o.x = state.homePos[i].x;
        o.y = state.homePos[i].y;
        o.radius = 3 * BOT_RADIUS;
        obs.push_back(o);
      }
    }

    for(int i = 0; i < state.awayDetected.size(); ++i)
    {
      if (state.awayDetected[i]) {
        obstacle o;
        o.x = state.awayPos[i].x;
        o.y = state.awayPos[i].y;
        o.radius = 3 * BOT_RADIUS;
        obs.push_back(o);
      }
    }
    Vector2D<int> point, nextWP, nextNWP;
    // use the mergescurver planner
    Navigation::MergeSCurve pathPlanner;
    Vector2D<int> botPos(state.homePos[botID].x, state.homePos[botID].y);
    pathPlanner.plan(botPos,
                      dpoint,
                      &nextWP,
                      &nextNWP,
                      &obs[0],
                      obs.size(),
                      botID,
                      true);
    return Navigation::waypointCommand(botID, state, nextWP, nextNWP, param.GoToPointP.finalslope, param.GoToPointP.align);
  } // goToPoint
}
