#!/usr/bin/env python3
"""
Manual control node — wsad keyboard input → XycarMotor

  w : speed  +5  (held → +5 every 0.05 s)
  s : speed  -5
  a : angle  -5  (left)
  d : angle  +5  (right)
  r : reset both angle and speed to 0
  Ctrl+C  : quit (터미널에서)

Speed  range : -50 ~ 50
Angle  range : -100 ~ 100

pygame 창에 포커스를 두고 키를 누르세요. 동시 입력 지원.
"""
import rclpy
from rclpy.node import Node
from xycar_msgs.msg import XycarMotor
import pygame

SPEED_STEP  = 1
ANGLE_STEP  = 8
SPEED_LIMIT = 50
ANGLE_LIMIT = 100
INTERVAL    = 0.05   # 20 Hz

W, H = 420, 300
BLACK  = (0, 0, 0)
WHITE  = (255, 255, 255)
GRAY   = (130, 130, 130)
BLUE   = (0, 180, 255)
GREEN  = (0, 230, 100)


class ManualControl(Node):

    def __init__(self):
        super().__init__('manual_control')
        self._pub = self.create_publisher(XycarMotor, 'xycar_motor', 10)
        self._speed = 0.0
        self._angle = 0.0

        pygame.init()
        self._screen = pygame.display.set_mode((W, H))
        pygame.display.set_caption('Manual Control')
        self._font  = pygame.font.SysFont('monospace', 16)
        self._fsmall = pygame.font.SysFont('monospace', 13)

        self.create_timer(INTERVAL, self._update)
        self.get_logger().info('Manual control ready — pygame 창에 포커스 후 키 입력')

    # ── display ───────────────────────────────────────────────────────────

    def _draw_bar(self, label, value, v_max, y, color):
        cx = W // 2
        bar_max = W // 2 - 30
        self._screen.blit(self._fsmall.render(label, True, (180, 180, 180)), (10, y - 8))
        pygame.draw.line(self._screen, (60, 60, 60), (cx, y - 10), (cx, y + 10), 1)
        fill = int(bar_max * abs(value) / v_max)
        x0 = cx if value >= 0 else cx - fill
        pygame.draw.rect(self._screen, color, (x0, y - 8, fill, 16))
        pygame.draw.rect(self._screen, (80, 80, 80), (cx - bar_max, y - 8, bar_max * 2, 16), 1)
        self._screen.blit(
            self._fsmall.render(f'{value:+.0f}', True, (220, 220, 220)),
            (cx + bar_max + 8, y - 8))

    def _draw(self):
        self._screen.fill(BLACK)
        title = self._font.render('Manual Control', True, WHITE)
        self._screen.blit(title, (W // 2 - title.get_width() // 2, 10))
        self._draw_bar('ANGLE', self._angle, ANGLE_LIMIT, 80,  BLUE)
        self._draw_bar('SPEED', self._speed, SPEED_LIMIT, 140, GREEN)

        guide = [('w', 'speed+'), ('s', 'speed-'), ('a', 'angle left'),
                 ('d', 'angle right'), ('r', 'reset'), ('C-c', 'quit')]
        for i, (k, desc) in enumerate(guide):
            x = 20 + (i % 2) * 200
            y = 210 + (i // 2) * 22
            self._screen.blit(self._fsmall.render(f'[{k}] {desc}', True, GRAY), (x, y))

        pygame.display.flip()

    # ── timer callback ────────────────────────────────────────────────────

    def _update(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                rclpy.shutdown()
                return
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_r:
                    self._speed = 0.0
                    self._angle = 0.0

        keys = pygame.key.get_pressed()
        if keys[pygame.K_w]:
            self._speed = min(self._speed + SPEED_STEP,  SPEED_LIMIT)
        if keys[pygame.K_s]:
            self._speed = max(self._speed - SPEED_STEP, -SPEED_LIMIT)
        if keys[pygame.K_a]:
            self._angle = max(self._angle - ANGLE_STEP, -ANGLE_LIMIT)
        if keys[pygame.K_d]:
            self._angle = min(self._angle + ANGLE_STEP,  ANGLE_LIMIT)

        msg = XycarMotor()
        msg.speed = float(self._speed)
        msg.angle = float(self._angle)
        self._pub.publish(msg)
        self._draw()

    # ── cleanup ───────────────────────────────────────────────────────────

    def destroy_node(self):
        pygame.quit()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ManualControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = XycarMotor()
        stop.speed = 0.0
        stop.angle = 0.0
        node._pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
