import pygame
from .core import * 
from .utils import *
import os
import pymunk
from ament_index_python.packages import get_package_share_directory

ASSETS_DIR = os.path.join(get_package_share_directory('simulation'), "assets")

 
class Game:
    def __init__(self):
        pygame.init()
        pygame.display.set_caption("Simulation")
        self.clock = pygame.time.Clock()
        self.screen = pygame.display.set_mode((kScreenWidth, kScreenHeight+kUIHeight))
        self.alpha_canvas_front = pygame.Surface((kScreenWidth, kScreenHeight), pygame.SRCALPHA)
        self.alpha_canvas_back = pygame.Surface((kScreenWidth, kScreenHeight), pygame.SRCALPHA)
        self.background = (pygame.transform.scale(pygame.image.load(os.path.join(ASSETS_DIR, "background.png")),(kScreenWidth, kScreenHeight))).convert() 
        self.blue_robot_sprite = (pygame.transform.scale(pygame.image.load(os.path.join(ASSETS_DIR, "blueRobotSprite.png")),(kRobotWidth *  kScreenWidth / kWorldWidth, kRobotHeight * kScreenHeight / kWorldHeight))).convert_alpha()
        self.yellow_robot_sprite = (pygame.transform.scale(pygame.image.load(os.path.join(ASSETS_DIR, "yellowRobotSprite.png")),(kRobotWidth *  kScreenWidth / kWorldWidth, kRobotHeight * kScreenHeight / kWorldHeight))).convert_alpha()
        self.initUI()
        self.space = pymunk.Space()
        self.space.damping = 0
        self.timer = 0

        self.initRobots()
        
        self.boxes = []
        for pos, angle in kBoxStartPositionAngle:
            self.boxes.extend(self.generateBoxSet(pos, angle))
            
        self.attics = []
        for pos in kAtticsPosition:
            self.attics.append(Attic(pos, 0, 0.2, 0.2, self.alpha_canvas_back))
        
        self.generateBorders(kBorderPosition)

    def initUI(self):
        self.ui_background = (pygame.transform.scale(pygame.image.load(os.path.join(ASSETS_DIR, "ui_background.png")),(kScreenWidth, kUIHeight))).convert()
        self.font = pygame.font.SysFont(None, 50)

    def getScore(self):
        blue, yellow = 0, 0
        for attic in self.attics:
            blue += 3*attic.blue_score
            yellow += 3*attic.yellow_score
            if attic.team == Team.BLUE:
                blue += 5
            if attic.team == Team.YELLOW:
                yellow += 5
        return blue, yellow
     
    def updateUI(self):
        blue, yellow = self.getScore()
        
        self.screen.blit(self.ui_background, (0, kScreenHeight))

        light_blue = (64, 172, 221, 255)
        yellow_score_surface = self.font.render(f"SCORE: {yellow}", True, kYellow)
        blue_score_surface = self.font.render(f"SCORE: {blue}", True, light_blue)
        self.screen.blit(yellow_score_surface, (180, kScreenHeight + kUIHeight / 2 - 15))
        self.screen.blit(blue_score_surface, (kScreenWidth / 2  + 200, kScreenHeight + kUIHeight / 2 - 15 ))

    def initRobots(self):
        team = Team.BLUE #temporary
        
        self.robotYellow = Robot(
            team = Team.YELLOW,
            position= (kRobotHeight / 2 + .2, kWorldHeight - kRobotWidth / 2 - .1), 
            angle= -pi/2, 
            width= kRobotWidth, 
            height= kRobotHeight, 
            canvas= self.screen,
            sprite = self.yellow_robot_sprite,
            space= self.space,
            mass=kRobotMass,
            wheel_distance=kWheelDistance,
            wheel_radius=kWheelRadius,
            control= (team == Team.YELLOW))
        
        Grabber(
            robot= self.robotYellow, 
            position=(kRobotHeight/ 2, 0), 
            angle=0, 
            width=kRobotWidth*0.8, 
            height=0.15, 
            canvas=self.alpha_canvas_front)
        
        Grabber(
            robot= self.robotYellow, 
            position= (-kRobotHeight / 2, 0), 
            angle= pi, 
            width= kRobotWidth*0.8, 
            height= 0.15, 
            canvas=self.alpha_canvas_front)

        self.robotBlue = Robot(
            team = Team.BLUE,
            position= (kWorldWidth - kRobotHeight / 2 - .2, kWorldHeight - kRobotWidth / 2 - .1), 
            angle= -pi/2, 
            width= kRobotWidth, 
            height= kRobotHeight, 
            canvas= self.screen,
            sprite = self.blue_robot_sprite,
            space= self.space,
            mass=kRobotMass,
            wheel_distance=kWheelDistance,
            wheel_radius=kWheelRadius,
            control= (team == Team.BLUE))
        Grabber(
            robot= self.robotBlue, 
            position=(kRobotHeight/ 2, 0), 
            angle=0, 
            width=kRobotWidth*0.8, 
            height=0.15, 
            canvas=self.alpha_canvas_front)
        
        Grabber(
            robot= self.robotBlue, 
            position= (-kRobotHeight / 2, 0), 
            angle= pi, 
            width= kRobotWidth*0.8, 
            height= 0.15, 
            canvas=self.alpha_canvas_front)

    def generateBorders(self, points: list[tuple[float]]):
        static_body = self.space.static_body
        n = len(points)
        for i in range(n-1):
            self.space.add(pymunk.Segment(static_body,points[i],points[i+1],kBorderRadius))
        self.space.add(pymunk.Segment(static_body,points[n-1],points[0],kBorderRadius))
    
    def generateBoxSet(self, position: Vector2, angle: float):
        cosa, sina = cos(angle), sin(angle)
        return [ 
            Box(
            position=((position.x + kBoxHeight/2) * cosa + (position.x + kBoxWidth * (i+.5)) * sina, (position.y + kBoxWidth * (i+.5)) * cosa + (position.y + kBoxHeight/2) * sina),
            angle=angle,
            width=kBoxWidth,
            height=kBoxHeight,
            team=team, 
            canvas=self.screen,
            space=self.space,
            depth=kBoxDepth,
            mass=kBoxMass) for i, team in enumerate(random.sample([Team.BLUE, Team.YELLOW] * 2,4))]
    
    def updateFromInput(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False

            for robot in [self.robotBlue, self.robotYellow]:
                if event.type == pygame.KEYDOWN:
                    if robot.control == True:         
                        if event.key == pygame.K_g:
                            if len(robot.grabbers[0].content) > 0:
                                robot.grabbers[0].release()                   
                            else:
                                robot.grabbers[0].grab(self.boxes)
                        if event.key == pygame.K_h:
                            if len(robot.grabbers[1].content) > 0:
                                robot.grabbers[1].release()
                            else:
                                robot.grabbers[1].grab(self.boxes)
                        if event.key == pygame.K_c:
                            for grabber in robot.grabbers:
                                grabber.sortColor()
                        if event.key == pygame.K_SPACE:
                            for box in self.boxes:
                                box.destroy()
                            self.boxes = []
                            for pos, angle in kBoxStartPositionAngle:
                                self.boxes.extend(self.generateBoxSet(pos,angle))
                if robot.control == True:
                    keys = pygame.key.get_pressed()
                    if keys[pygame.K_q]:
                        robot.motor_left.setSpeed(2)
                    elif  keys[pygame.K_a]:
                        robot.motor_left.setSpeed(-2)
                    else:
                        if(robot.actionTime == 0):
                            robot.motor_left.setSpeed(0)
                    if keys[pygame.K_e]:
                        robot.motor_right.setSpeed(2)
                    elif  keys[pygame.K_d]:
                        robot.motor_right.setSpeed(-2)
                    else:
                        if(robot.actionTime == 0):
                            robot.motor_right.setSpeed(0.)
                    if keys[pygame.K_UP]:
                         robot.translate(2,2)
                    if keys[pygame.K_DOWN]:
                        robot.translate(-2,2)
                    if keys[pygame.K_LEFT]:
                        robot.rotate(pi/4,0.5)
                    if keys[pygame.K_RIGHT]:
                        robot.rotate(-pi/4,0.5)
        return True

    def updatePhysics(self, dt):
        sub_steps = 3 
        for s in range(1, sub_steps + 1):
            self.robotYellow.updateMotor(dt/sub_steps)
            self.robotBlue.updateMotor(dt/sub_steps)
            self.space.step(dt/sub_steps) 
            
    def drawAndUpdateAttics(self):
        self.alpha_canvas_back.fill((0, 0, 0, 0))
        for attic in self.attics:
            attic.updateTeam(self.boxes)
            attic.draw()
        self.screen.blit(self.alpha_canvas_back, (0, 0))
            
    def drawBoxes(self):
        for box in self.boxes:
                box.draw()
                
    def drawAndUpdateRobots(self, dt: float):
        self.alpha_canvas_front.fill((0, 0, 0, 0))
        for robot in [self.robotBlue, self.robotYellow]:
            robot.drawPath()
            
        for robot in [self.robotBlue, self.robotYellow]:
            robot.updateAction(dt)
            robot.draw()
            
        
        self.screen.blit(self.alpha_canvas_front, (0, 0))

    def step(self):
        dt = self.clock.tick(60)/1000
        self.timer += dt
        self.screen.blit(self.background, (0, 0))
        self.drawAndUpdateAttics()
        self.drawBoxes()
        self.drawAndUpdateRobots(dt)

            
        self.updatePhysics(dt)
        self.updateUI()
            
        pygame.display.flip()
    
    def start(self):
        running = True
        while running:
            running = self.updateFromInput() 
            self.step()

    def quit(self):
        pygame.quit()
        
        
        
