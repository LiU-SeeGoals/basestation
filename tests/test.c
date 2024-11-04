#include "parsed_vision/parsed_vision.pb-c.h"
#include <stdio.h>

void print_frame(int ball_px, int ball_py, int ball_vx, int ball_vy, size_t robot_count) {
  ParsedFrame frame;
  parsed_frame__init(&frame);
  Ball ball;
  ball__init(&ball);
  Vector2D pos;
  vector2_d__init(&pos);
  pos.x = ball_px;
  pos.y = ball_py;
  Vector2D vel;
  vector2_d__init(&vel);
  vel.x = ball_vx;
  vel.y = ball_vy;
  ball.pos = &pos;
  ball.vel = &vel;
  frame.ball = &ball;
  Robot robots[16];
  Robot* rob[16];
  for (int i = 0; i < robot_count; ++i) {
    robot__init(&robots[i]);
    robots[i].robot_id = i;
    Vector3D pos;
    Vector3D vel;
    vector3_d__init(&pos);
    vector3_d__init(&vel);
    robots[i].pos = &pos;
    robots[i].vel = &vel;

    robots[i].vel->x = 12 + i;
    robots[i].vel->y = 15 - i;
    robots[i].pos->x = 5;
    robots[i].pos->y = 2 * i;
    rob[i] = &robots[i];
  }
  frame.n_robots = robot_count;
  frame.robots = rob;
  uint8_t buf[256];
  size_t size = parsed_frame__pack(&frame, buf);

  const char* hex = "0123456789abcdef";
  for (int i = 0; i < size; ++i) {
    printf("%d,", buf[i]);
  }
}

int main() {
  print_frame(123, -456, 0, 0, 2);
  return 0;
}
