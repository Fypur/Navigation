import random
from pygame import Vector2

kScreenWidth = 1080
kScreenHeight = 720
kUIHeight = 90
kWorldWidth = 3
kWorldHeight = 2

#box setup
kBoxWidth = 0.05
kBoxHeight = 0.15
kBoxDepth = 0.03
kBoxMass = 0.5
kYellow = (247, 181, 0, 255)
kBlue = (0, 91, 140, 255)

#Robot setup
kRobotWidth = 0.303
kRobotHeight = 0.297
kRobotMass = 1e10
kWheelRadius = .035
kWheelDistance = 0.222 #from one wheel to the other

# kBoxStartPositionAngle = []
kBoxStartPositionAngle = [
               (Vector2(0.1, 1.1), 0),
               (Vector2(0.1, 0.3), 0),
               (Vector2(1, 0.1), 3.14 / 2),
               (Vector2(1.05, 0.725), 3.14 / 2),
               (Vector2(1.75, 0.725), 3.14 / 2),
               (Vector2(1.8, 0.1), 3.14 / 2),
               (Vector2(2.75, 1.1), 0),
               (Vector2(2.75, 0.3), 0),
              ]

kAtticsPosition = [Vector2(0.1, 0.8),
                      Vector2(0.8, 0.8),
                      Vector2(1.5, 0.8),
                      Vector2(2.2, 0.8),
                      Vector2(2.9, 0.8),
                      Vector2(0.7, 0.1),
                      Vector2(1.5, 0.1),
                      Vector2(2.3, 0.1),
                      Vector2(1.25, 1.45),
                      Vector2(1.75, 1.45),
              ]

kBorderRadius = 0.1
kBorderPosition = [(0-kBorderRadius,0-kBorderRadius),
          (0-kBorderRadius,kWorldHeight+kBorderRadius),
          (0.600+kBorderRadius,kWorldHeight+kBorderRadius),
          (0.600+kBorderRadius,kWorldHeight-0.450+kBorderRadius),
          (kWorldWidth-0.600-kBorderRadius,kWorldHeight-0.450+kBorderRadius),
          (kWorldWidth-0.600-kBorderRadius,kWorldHeight+kBorderRadius),
          (kWorldWidth+kBorderRadius,kWorldHeight+kBorderRadius),
          (kWorldWidth+kBorderRadius,0-kBorderRadius),
          ]

def canvasToWorld(v):
    return (v.x / kScreenWidth * kWorldWidth, kWorldHeight - v.y / kScreenHeight * kWorldHeight)

def worldToScreen(v):
    return (v.x / kWorldWidth * kScreenWidth, kScreenHeight - v.y / kWorldHeight * kScreenHeight)
