#include "motion_internal.h"
#include "tb6612.h"

static const motion_hooks_t no_hooks;
const motion_hooks_t *motion_hooks = &no_hooks;

void motion_init(const motion_hooks_t *hooks) { motion_hooks = hooks ? hooks : &no_hooks; }

void motion_step(float dt)
{
  move_step(dt);
  straight_step(dt);
  rotate_step(dt);
}

void motion_stop(void)
{
  move_stop();
  straight_stop();
  rotate_stop();
  tb6612_stop_all();
}

int motion_busy(void) { return move_active() || straight_active() || rotate_active(); }

motion_mode_t motion_mode(void)
{
  if (rotate_active()) return MOTION_ROTATE;
  if (straight_active()) return MOTION_STRAIGHT;
  if (vel_active()) return MOTION_VEL;
  if (move_active()) return MOTION_MOVE;
  return MOTION_IDLE;
}
