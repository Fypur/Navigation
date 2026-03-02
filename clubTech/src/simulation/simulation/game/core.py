from abc import abstractmethod,ABC
from math import cos, sin, pi
import math
from pygame import Vector2
from pygame.time import Clock
import pygame
import pymunk
from enum import Enum
from .utils import *

class Component(ABC):
    def __init__(self, position:Vector2, angle:float, width:float, height:float, canvas:pygame.surface):
        self.pos = Vector2(position)
        self.width = width
        self.height = height
        self.angle = angle
        self.canvas = canvas
        self.parent = None
    
    def worldToRelativeCoordinates(self, v:Vector2):
        x, y = v
        cosa, sina = cos(self.angle), sin(self.angle)
        x, y = x - self.pos.x, y - self.pos.y
        rx = cosa * x + sina * y 
        ry = cosa * y - sina * x           
        return Vector2(rx,ry)
    
    def relativeToWorldCoordinates(self, v:Vector2):
        x,y = v
        cosa, sina = cos(self.angle), sin(self.angle)
        rx = cosa * x - sina * y 
        ry = cosa * y + sina * x   
        return Vector2(rx + self.pos.x, ry + self.pos.y)
    
    def getPoints(self):
        hh, hw = self.height / 2, self.width / 2        
        return [
            self.relativeToWorldCoordinates(Vector2(-hh, -hw)),
            self.relativeToWorldCoordinates(Vector2(-hh, hw)),
            self.relativeToWorldCoordinates(Vector2(hh, hw)),
            self.relativeToWorldCoordinates(Vector2(hh, -hw))]
    
    def hitbox(self):
        cosa, sina = cos(self.angle), sin(self.angle)
        width = abs(self.width*cosa) + abs(self.height*sina)
        height = abs(self.width*sina) + abs(self.height*cosa)
        return width,height

    def aabb_collision(self, other):
        self_width, self_height = self.hitbox()
        other_width, other_height = other.hitbox()
        x_condition = (abs(self.pos.x - other.pos.x) < (self_height + other_height) / 2)
        y_condition = (abs(self.pos.y - other.pos.y) < (self_width + other_width) / 2)
        return x_condition and y_condition
    
    def sat_collision(self, other):
        cosa, sina = cos(self.angle), sin(self.angle)
        cosb, sinb = cos(other.angle), sin(other.angle)
        
        axis = [Vector2(cosa, sina), Vector2(-sina, cosa), Vector2(cosb, sinb), Vector2(-sinb, cosb)]
        self_points = self.getPoints()
        other_points = other.getPoints()
        for a in axis:
            self_projected_points = [a.dot(points) for points in self_points]
            self_min, self_max = min(self_projected_points), max(self_projected_points)
            other_projected_points = [a.dot(points) for points in other_points]
            other_min, other_max = min(other_projected_points), max(other_projected_points)
            if (self_max < other_min) or (other_max < self_min):
                return False
        return True
    
    def detect_collision(self, other):
        return self.aabb_collision(other) and self.sat_collision(other)

    @abstractmethod
    def draw(self):
        pass
         
class PhysicsComponent(Component):
    def __init__(self, position: Vector2, angle: float, width: float, height: float, canvas:pygame.Surface,space: pymunk.space, mass: float):
        super().__init__(position, angle, width, height, canvas)
        self.mass = mass
        self.space = space
        self.body = pymunk.Body(self.mass,self.getMomentum())
        self.body.body_type = pymunk.Body.DYNAMIC
        hw, hh = self.width/2,self.height/2
        points = [(-hh, -hw), (-hh, hw), (hh, hw), (hh, -hw)] #in relative coordinates
        self.shape = pymunk.Poly(self.body,points)
        self.space.add(self.shape,self.body)
        self.body.position = (self.pos.x,self.pos.y)
        self.body.angle = angle
            
    def getMomentum(self):
        return pymunk.moment_for_box(self.mass, (self.height, self.width))
    
    def syncPositionAngle(self):
        """ makes sure compomnent position is synced to Physics position"""
        self.angle = self.body.angle
        self.pos = self.body.position
    
    def setAngle(self, angle: float):
        self.angle = angle
        self.body.angle = angle
        
    def setPosition(self, position: Vector2):
        self.pos = position
        self.body.position = (position.x,position.y)
        
class Team(Enum):
    YELLOW = 0
    BLUE = 1

class Box(PhysicsComponent):
    def __init__(self, position: Vector2, width: float, height: float, team: Team, canvas: pygame.surface, space: pymunk.space, depth: float=0.03,is_grabed: float=False,angle: float=0.,mass=0.1):
        super().__init__(position, angle, width, height, canvas,space, mass)
        self.team = team
        self.depth = depth
        self.is_grabed = is_grabed
        
    def draw(self):
        self.syncPositionAngle()
        if not self.is_grabed:
            if self.team == Team.YELLOW :
                pygame.draw.polygon(self.canvas, kYellow, [worldToScreen(Vector2(x, y)) for (x, y) in self.getPoints()])
            elif self.team == Team.BLUE:
                pygame.draw.polygon(self.canvas, kBlue, [worldToScreen(Vector2(x, y)) for (x, y) in self.getPoints()])
        else:
            if self.team == Team.YELLOW :
                pygame.draw.polygon(self.canvas, kYellow, [worldToScreen(self.parent.parent.relativeToWorldCoordinates(Vector2(x, y))) for (x, y) in self.getPoints()])
            elif self.team == Team.BLUE:
                pygame.draw.polygon(self.canvas, kBlue, [worldToScreen(self.parent.parent.relativeToWorldCoordinates(Vector2(x, y))) for (x, y) in self.getPoints()])
    
    def getGrabed(self,parent, pos:Vector2, angle: float):
        self.is_grabed = True
        self.body.body_type = pymunk.Body.KINEMATIC
        self.height, self.depth = self.depth, self.height
        self.parent = parent
        self.setPosition(pos)
        self.setAngle(angle)
        
    def getReleased(self, pos: Vector2):
        self.is_grabed = False
        self.body.body_type = pymunk.Body.DYNAMIC
        self.body.mass = self.mass
        self.body.moment = self.getMomentum()
        self.height, self.depth = self.depth, self.height
        self.setPosition(pos)
        self.setAngle(self.angle + self.parent.parent.angle)
        self.parent = None
    
    def getTeam(self):
        if self.team == Team.BLUE:
            return "BLUE"
        else:
            return "YELLOW"
        
    def destroy(self):
        self.space.remove(self.shape)
        self.space.remove(self.body)
        self.parent = None
        self.is_grabed = False

class Attic(Component):
    def __init__(self, position, angle, width, height, canvas):
        super().__init__(position, angle, width, height, canvas)
        self.team = None
        self.yellow_score = 0
        self.blue_score = 0
    
    def draw(self):
        yellow = (247, 181, 0, 100)
        blue = (0, 91, 140, 100)
        color = (171, 175, 178, 100)
        
        if self.team is not None:
            if self.team == Team.YELLOW :
                color = yellow
            elif self.team == Team.BLUE:
                color = blue
        pygame.draw.polygon(self.canvas, color, [worldToScreen(Vector2(x, y)) for (x, y) in self.getPoints()])   
        
    def updateTeam(self, boxList: list[Box]):
        blue = 0
        yellow = 0
        for box in boxList:
            if self.detect_collision(box):
                if box.team == Team.BLUE:
                    blue += 1
                else:
                    yellow += 1
        
        self.blue_score = blue
        self.yellow_score = yellow
        if(blue == yellow):
            self.team = None
        elif(yellow > blue):
            self.team = Team.YELLOW
        else:
            self.team = Team.BLUE
                
class Grabber(Component):
    def __init__(self, robot, position:Vector2, angle:float, width:float, height:float, canvas:pygame.surface, grab_angle_limit:float=pi/8):
        super().__init__(position,angle,width,height,canvas)
        self.parent = robot
        self.parent.grabbers.append(self)
        self.grab_angle_limit=grab_angle_limit
        self.content = []
    
    def canGrab(self, box: Box):
        limit = self.height
        box_x, box_y = self.worldToRelativeCoordinates(self.parent.worldToRelativeCoordinates(Vector2(box.pos.x, box.pos.y)))
        is_close_enough = (abs(box_x) < limit and abs(box_y) < self.width / 2)
        is_right_angle = (abs(cos(self.angle + self.parent.angle) * cos(box.angle) + sin(self.angle + self.parent.angle) * sin(box.angle)) > cos(self.grab_angle_limit))
        return is_close_enough and is_right_angle  
    
    def grab(self, boxList: list[Box]):
        if len(self.content) >= 4:
            return False
         
        grabed = [box for box in boxList if self.canGrab(box)]
        
        if len(grabed) == 4:
            step = -self.width / 2
            for box in grabed:
                box.getGrabed(self,self.relativeToWorldCoordinates(Vector2(box.depth / 2, step + box.width / 2)), self.angle)
                step += self.width / 4
            self.content = grabed   
            return True
        return False

    def release(self):
        if len(self.content) == 0:
            return
        step = -2 * self.content[0].width + self.content[0].width / 2
        for box in self.content:
            box.getReleased(self.parent.relativeToWorldCoordinates(self.relativeToWorldCoordinates(Vector2(box.depth / 2, step))))
            step += box.width
        self.content = []

    def draw(self):
        grey = (171, 175, 178, 150)
        cosa, sina = cos(self.angle), sin(self.angle)
        pygame.draw.polygon(self.canvas, grey, [worldToScreen(self.parent.relativeToWorldCoordinates(point + (self.height / 2) * Vector2(cosa, sina))) for point in self.getPoints()])
        
    def sortColor(self):
        for box in self.content:
            box.team = self.parent.team

class Motor:
    def __init__(self, wheel_radius: float):
        self.angular_speed = 0
        self.wheel_radius = wheel_radius
    
    def setSpeed(self, speed: float):
        """ is an angular speed in tour/sec """
        self.angular_speed = speed
    
    def setVelocity(self, speed: float):
        """ is an angular speed in tour/sec """
        self.angular_speed = speed/(2 * pi * self.wheel_radius)
        
    def getAngularSpeed(self):
        return self.angular_speed * 2 * pi

    def getSpeed(self):
        return 2 * pi * self.wheel_radius * self.angular_speed
           
class Robot(PhysicsComponent):
    def __init__(self,team:Team, position: Vector2, angle: float, width: float, height: float, canvas: pygame.Surface, sprite: pygame.image,  control: float, space: pymunk.space, mass:float, speed:float = 0., angular_speed:float=0., acceleration:float = 0., wheel_distance: float=kWheelDistance, wheel_radius: float=0.035):
        super().__init__(position, angle, width, height, canvas, space, mass)
        self.team = team
        self.sprite = sprite
        self.control = control
        self.body.velocity = speed*pymunk.Vec2d(cos(self.angle),sin(self.angle))
        self.body.angular_velocity = angular_speed
        self.acceleration = Vector2(acceleration)
        self.wheel_distance = wheel_distance
        self.motor_left = Motor(wheel_radius)
        self.motor_right = Motor(wheel_radius)
        self.body.body_type = pymunk.Body.DYNAMIC
        self.path = []
        self.grabbers = []
        self.actionTime = 0
        self.actionStartTime = 0
        self.body.damping = 0
                
    def draw(self):
        self.syncPositionAngle()

        rotated_sprite = pygame.transform.rotate(self.sprite, math.degrees(self.body.angle) - 90)
        pos_screen = worldToScreen(self.body.position)
        rect = rotated_sprite.get_rect(center=pos_screen)
        self.canvas.blit(rotated_sprite, rect.topleft)
        # pygame.draw.polygon(self.canvas, grey, [worldToScreen(Vector2(x,y)) for (x,y) in self.getPoints()])
        
        # cosa, sina = cos(self.angle), sin(self.angle)
        # pygame.draw.line(self.canvas, red, worldToScreen(Vector2(self.pos.x + self.height / 2 * cosa, self.pos.y + self.height / 2 * sina)), worldToScreen(Vector2(self.pos.x + cosa * (self.height / 2 +  0.1), self.pos.y + sina * (self.height / 2 + 0.1))),2)
        for g in self.grabbers:
            g.draw()
        
        
    def drawPath(self):
        color = (64, 172, 221, 255) if self.team == Team.BLUE else (255, 225, 100, 255)
        if len(self.path) > 0:
            pygame.draw.line(self.canvas, color, worldToScreen(self.body.position), worldToScreen(self.path[0]), 4)
            for i in range(len(self.path)-1):
                pygame.draw.line(self.canvas, color, worldToScreen(self.path[i]), worldToScreen(self.path[i+1]), 4)
    
    def updateMotor(self, dt):
        velocity = (self.motor_left.getSpeed() + self.motor_right.getSpeed()) / 2
        angular_velocity = (self.motor_right.getSpeed() - self.motor_left.getSpeed()) / self.wheel_distance
        
        if abs(velocity) > 0:
            target_speed = self.body.rotation_vector * velocity
            current_speed = self.body.velocity
            speed_error = target_speed - current_speed*self.body.damping
            forward_force = speed_error/dt*self.mass
            self.body.apply_force_at_world_point(forward_force, self.body.position) 
    
        if abs(angular_velocity) > 0:
            angular_error = angular_velocity-self.body.angular_velocity*self.body.damping
            self.body.torque = angular_error/dt*self.body.moment
    
    def setActionTime(self, time: float):
        self.actionTime = time
        self.actionStartTime = 0
        
    def translate(self, speed:float, time: float):
        self.setActionTime(time)
        self.motor_left.setSpeed(speed)
        self.motor_right.setSpeed(speed)
      
    def rotate(self, angle: float, time: float):
        # calib_factor = (math.pi/2) / (math.pi/2 + math.radians(0.52))
        if time > 0:
            speed = angle*self.wheel_distance/(time*2*pi*self.motor_left.wheel_radius)
            self.setActionTime(time)
            self.motor_left.setSpeed(-speed)
            self.motor_right.setSpeed(speed)
          
    def updateAction(self, dt: float):
        if self.actionTime > 0:
            self.actionStartTime += dt

            if self.actionStartTime >= self.actionTime:
                self.actionTime = 0
                self.actionStartTime = 0
                self.motor_left.setSpeed(0)
                self.motor_right.setSpeed(0)

        