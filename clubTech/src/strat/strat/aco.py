from matplotlib.text import Annotation
import numpy as np
from typing import Literal
from collections import defaultdict
from typing import Any, List, Optional, Iterable, Callable, Dict, Set, TypeVar, Generator

sigmoid = lambda z: 1/(1+(np.exp((-z))))

class ACO:
    defaultHeuristic: Callable[[float, list[int], int], float] = lambda pheromone, visited, j: pheromone

    def __init__(self,
                 evalRoute: Callable[[List[int]], float],
                 n: Optional[int] = None,
                 heuristic: Callable[[float, list[int], int], float] = defaultHeuristic,
                 endPoints: tuple[list[int], list[int]] = ([], []),
                 exists: Optional[list[list[int]]] = None,
                 q=128,
                 antCount=32,
                 explorationRate=.05,
                 stopProbability=.1,
                 evaporation=0.1):
        
        nValues = set(([n] if n else []) + [len(e) for e in [exists] if e])

        if len(nValues) != 1:
            raise RuntimeError('no way to determine `n`' if len(nValues) == 0 else 'cost value does not match')
        
        self.n = list(nValues)[0] or -1 # never -1, just for type

        self.endPoints = endPoints
        self.heuristic = heuristic

        self.exists = exists or [[j for j in range(self.n) if i != j] for i in range(self.n)]
        self.evalRoute: Callable[[List[int]], float] = evalRoute

        self.q = q
        self.antCount = antCount
        self.explorationRate = explorationRate
        self.stopProbability = stopProbability
        self.evaporation = evaporation

        self.pheromones = np.ones((self.n, self.n))
        self.bestRoute: List[int] = []
        self.bestCost = float('inf')
        self.inactivityCounter = 0

    @staticmethod
    def getDist (points: np.ndarray):
        return np.linalg.norm(points[:, None, :] - points[None, :, :], axis=2)


    def chooseNext(self, visited: List[int], visitedMask: list[bool]) -> Optional[int]:
        current = visited[-1]
        allowed = [j for j in self.exists[current] if not visitedMask[j] and j not in self.endPoints[1]]

        if not allowed:
            return None

        if np.random.random() < self.explorationRate:
            return np.random.choice(allowed)

        probs = np.array([self.heuristic(self.pheromones[current][j], visited, j) for j in allowed])
        probs += 1e-12
        probs /= probs.sum()
        return np.random.choice(allowed, p=probs)

    def buildRandomRoute(self) -> Optional[List[int]]:
        visited = [np.random.randint(self.n)] if self.endPoints[0] is None else list(self.endPoints[0])
        visitedMask = [j in visited for j in range(self.n) ]
        maxLength = self.n - len(self.endPoints[1])

        while len(visited) < maxLength:
            # if np.random.random() < self.stopProbability:
            #     break

            next = self.chooseNext(visited, visitedMask)

            if next is None:
                return None
            
            visitedMask[next] = True
            visited.append(next)

        return visited + self.endPoints[1]

    def useBestRoute(self, bestRoute: List[int]):
        n = len(bestRoute)
        startN, endN = len(self.endPoints[0]), len(self.endPoints[1])
        middleRoute = bestRoute[startN:n-endN]
        middleLen = len(middleRoute)

        unused = [ e for e in range(self.n) if e not in bestRoute]

        prefix = bestRoute[:startN]
        suffix = bestRoute[n-endN:]

        # swapLevel = 1..middleLen-1
        for swapLevel in range(1, middleLen):
            # i = 0..middleLen-swapLevel-1
            for i in range(middleLen - swapLevel):
                j = i + swapLevel

                # yield new route
                yield prefix + middleRoute[:i] + middleRoute[i:j+1][::-1] + middleRoute[j+1:] + suffix

        for i in range(startN, startN + middleLen):
            yield bestRoute[:i] + bestRoute[i+1:]

        for i in range(startN, startN + middleLen):
            for e in unused:
                yield bestRoute[:i] + [e] + bestRoute[i:]

    def updateBestRoute(self, route: Optional[List[int]] = None, cost: Optional[float] = None):
        if route is None:
            route = self.bestRoute

        if cost is None:
            cost = self.evalRoute(route)

        # print(self.bestCost, '->', cost)

        if route != self.bestRoute:
            self.inactivityCounter = 0


        self.bestCost = cost
        self.bestRoute = route

    def __iter__(self):
        allRoutesWithCosts: List[tuple[List[int], float, float]] = []

        def registerRoute(route: List[int], pheromoneCoef = 1.):
            cost = self.evalRoute(route)

            allRoutesWithCosts.append((route, cost, pheromoneCoef))

            if cost < self.bestCost:
                self.updateBestRoute(route, cost)

                return True
            return False
        
        # initialize best route
        while True:
            route = self.buildRandomRoute()

            if route:
                registerRoute(route)
                yield route
                break

        
        while True:
            seenRoutes: set[tuple[int, ...]] = set()
            allRoutesWithCosts = []

            while True:
                route = self.buildRandomRoute()

                if not route:
                    continue

                routeKey = tuple(route)

                if routeKey in seenRoutes:
                    continue

                seenRoutes.add(routeKey)
                registerRoute(route)
                break

            if self.inactivityCounter == 1:
                while True:
                    for route in self.useBestRoute(self.bestRoute):
                        if registerRoute(route, .001):
                            break
                    else:
                        break

            self.pheromones *= 1 - self.evaporation
            self.inactivityCounter += 1

            if self.inactivityCounter % self.n == 0:
                # self.pheromones = self.pheromones.max() - self.pheromones
                self.pheromones = np.ones_like(self.pheromones) / 10
            else:
                for route, cost, coef in allRoutesWithCosts:
                    deposit = coef * self.q / cost

                    npRoute = np.array(route)
                    a = npRoute[:-1]
                    b = npRoute[1:]

                    self.pheromones[a, b] += deposit
                    self.pheromones[b, a] += deposit

            yield self.bestRoute

    
    def refresh (self):
        self.updateBestRoute()


class VisualACO(ACO):
    def __init__(self,
                 points: np.ndarray,
                 evalRoute,

                 names: dict[int, str] = {},
                 colors: Optional[list[tuple[int, int, int]]] = None,
                 n: Optional[int] = None,
                 heuristic: Callable[[float, list[int], int], float] = ACO.defaultHeuristic,
                 endPoints: tuple[list[int], list[int]] = ([], []),
                 exists: Optional[list[list[int]]] = None,
                 q=128,
                 antCount=32,
                 explorationRate=.05,
                 stopProbability=.1,
                 evaporation=0.1,
                 showPheromones=False,
                 figsize = (16, 9)):

        super().__init__(
            n=n,
            heuristic=heuristic,
            endPoints=endPoints,
            antCount=antCount,
            q=q,
            exists=exists,
            evalRoute=evalRoute,
            explorationRate=explorationRate,
            stopProbability=stopProbability,
            evaporation=evaporation,
        )

        self.points = points
        self.showPheromones = showPheromones

        from matplotlib import pyplot as plt
        
        self.fig, self.ax = plt.subplots(figsize=figsize)
        self.scatter = self.ax.scatter(points[:, 0], points[:, 1], color=colors)

        self.textNodes = [self.ax.text(x, y, names.get(idx, str(idx)), color=colors[idx] if colors else 'blue', fontsize=10) for idx, (x, y) in enumerate(points)]

        self.logText = self.ax.text(
            0.01, 0.99,
            "",
            transform=self.ax.transAxes,
            fontsize=10,
            color="darkgreen",
            verticalalignment="top",
            bbox=dict(boxstyle="round", facecolor="white", alpha=0.8)
        )

        self.bestLineCollection = [self.ax.plot([], [], color=(0, 0, 0, 0.0), linewidth=2)[0] for _ in range(self.n)]

        self.edgeLines = {}
        if self.showPheromones:
            for i in range(self.n):
                for j in self.exists[i]:
                    if j < i: continue

                    line, = self.ax.plot(
                        [self.points[i, 0], self.points[j, 0]],
                        [self.points[i, 1], self.points[j, 1]],
                        color=(0, 0, 1, 0.0),  # start invisible
                        linewidth=1
                    )
                    self.edgeLines[(i, j)] = line
        
        plt.axis('off')
        plt.ion()
        plt.show()

    def print(self, msg: str):
        self.logText.set_text(msg)
        self.fig.canvas.draw_idle()

    def drawBestRoute (self):
        for line in self.bestLineCollection:
            line.remove()
        self.bestLineCollection = []

        for i in range(len(self.bestRoute) - 1):
            a, b = self.bestRoute[i], self.bestRoute[i + 1]
            line, = self.ax.plot(
                [self.points[a, 0], self.points[b, 0]],
                [self.points[a, 1], self.points[b, 1]],
                color=(0, 0, 0, 1),
                linewidth=2
            )
            self.bestLineCollection.append(line)

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def refresh (self):
        self.updateBestRoute()

        for route in self.useBestRoute(self.bestRoute):
            cost = self.evalRoute(route)

            if cost < self.bestCost:
                self.updateBestRoute(route, cost)

        for (x, y), text in zip(self.points, self.textNodes):
            text.set_x(x)
            text.set_y(y)

        for (i, j), line in self.edgeLines.items():
            line.set_data(
                [self.points[i, 0], self.points[j, 0]],
                [self.points[i, 1], self.points[j, 1]],
            )

        # self.ax.scatter(self.points[:, 0], self.points[:, 1], color=colors)
        self.scatter.set_offsets(self.points)

        self.drawPheromones()
        self.drawBestRoute()


    def drawPheromones(self):
        maxPheromone = self.pheromones.max()
        if maxPheromone <= 0:
            return

        for (i, j), line in self.edgeLines.items():
            alphaVal = self.pheromones[i, j] / maxPheromone
            line.set_alpha(alphaVal)
            line.set_visible(alphaVal > 0.01)


        self.drawBestRoute()
        self.fig.canvas.draw_idle()

    def updateBestRoute(self, route: Optional[List[int]] = None, cost: Optional[float] = None):
        super().updateBestRoute(route, cost)
        # self.drawBestRoute()


    def __iter__(self):
        for route in super().__iter__():
            if self.inactivityCounter == 1:
                continue

            self.drawPheromones()
            self.drawBestRoute()

            yield route


def calculateDist (points: np.ndarray):
    dist = ACO.getDist(points)
    distCoefs = 1 / (dist + np.finfo(float).eps) ** 6

    BA = points[:, None, None, :] - points[None, :, None, :]
    BC = points[None, None, :, :] - points[None, :, None, :]

    dot = np.einsum('abcd,abcd->abc', BA, BC)
    norms = np.linalg.norm(BA, axis=-1) * np.linalg.norm(BC, axis=-1)
    cos = -dot / (norms + np.finfo(float).eps)
    angleDist = np.arccos(np.clip(cos, -1.0, 1.0))
    angleCoefs = 1 / (angleDist + .01) ** 2

    return dist, distCoefs, angleDist, angleCoefs

class ACORobotState:
    def __init__(self,
                 cost: float,
                 partitionMap: Dict[int, Set[int]],
                 types: list[int],
                 usedPoints: set[int],
                 kaplas = 0
    ) -> None:
        
        self.points = 0
        self.maxKaplas = 1
        self.grabPoints = 1e2
        self.releasePoints = 1e3

        self.kaplas = kaplas
        self.cost = cost
        self.types = types
        self.usedPoints = usedPoints
        self.partitionMap = partitionMap

    def copy (self):
        return ACORobotState(
            kaplas=self.kaplas,
            cost=self.cost,
            types=self.types,
            usedPoints=self.usedPoints.copy(),
            partitionMap=self.partitionMap
        )
    
    def canGrab (self):
        return self.kaplas != self.maxKaplas
    
    def canDrop (self):
        return self.kaplas != 0
    
    def run (self, destination: int, achieved = True):
        if destination in self.usedPoints:
            return

        if achieved:
            match self.types[destination]:
                case 0:
                    if self.canGrab():
                        self.kaplas += 1
                        self.cost -= self.grabPoints

                case 1:
                    if self.canDrop():
                        self.kaplas -= 1
                        self.cost -= self.releasePoints - self.grabPoints
                        self.points += 1

        for e in self.partitionMap.get(destination, { destination }):
            self.usedPoints.add(e)

def getVecAtAngle (theta: float):
    c, s = np.cos(theta), np.sin(theta)
    r = np.array(((c, -s), (s, c)))

    return r.dot(np.array((1, 0)))

def makeUnique (l):
    s = set()

    unique = []

    for e in l:
        if e in s:
            continue

        unique.append(e)
        s.add(e)

    return unique

T = TypeVar("T")
U = TypeVar("U")

def partition(
    l: Iterable[T],
    f: Callable[[T], U]
) -> Dict[T, Set[T]]:
    by_value = defaultdict(set)

    for e in l:
        by_value[f(e)].add(e)

    return {e: by_value[f(e)] for e in l}

def getOffsetAtAngle (theta: float):
    return np.array([[np.cos(theta), -np.sin(theta)], 
                    [np.sin(theta),  np.cos(theta)]]) @ np.array([ -1, 0 ])

def acoInit (
        rawKGrabBoxPositionAngle = [
            ((0.175,  1.2), np.pi),
            ((0.175,  0.4), np.pi),
            ((1.10086, 0.1751), -1/2 * np.pi),
            ((1.90149, 0.1751), -1/2 * np.pi),
            ((1.1509,  0.8006), 1/2 * np.pi),
            ((1.1509,  0.8006), -1/2 * np.pi),
            ((1.85145, 0.8006), 1/2 * np.pi),
            ((1.85145, 0.8006), -1/2 * np.pi),
            ((2.825,  1.2), 0.0),
            ((2.825,  0.4), 0.0)
        ],
        rawKAtticsPosition = [
            ((0.1, 0.8), np.pi),

            ((0.8, 0.8), -1/2 * np.pi),
            ((0.8, 0.8), 0.0),
            ((0.8, 0.8), 1/2 * np.pi),

            ((1.5, 0.8), -1/2 * np.pi),
            ((1.5, 0.8), 1/2 * np.pi),

            ((2.2, 0.8), -1/2 * np.pi),
            ((2.2, 0.8), np.pi),
            ((2.2, 0.8), 1/2 * np.pi),

            ((2.9, 0.8), 0.0),

            ((0.7, 0.1), -1/2 * np.pi),
            ((1.5, 0.1), -1/2 * np.pi),
            ((2.3, 0.1), -1/2 * np.pi),

            ((1.25, 1.45), 1/2 * np.pi),
            ((1.75, 1.45), 1/2 * np.pi),
        ]
):
    getKGrabBoxPositionAngle = lambda offset: [(tuple(np.array(p) + offset * getOffsetAtAngle(t)), t) for p, t in rawKGrabBoxPositionAngle]
    getKAtticsPosition = lambda offset: [(tuple(np.array(p) + offset * getOffsetAtAngle(t)), t) for p, t in rawKAtticsPosition]

    addUseless = False
    kRobotWidth = 0.303
    kRobotHeight = 0.297
    kBoxHeight = 0.15
    kWorldWidth = 3
    kWorldHeight = 2
    nid = (kWorldWidth - kRobotHeight / 2 - .2, kWorldHeight - kRobotWidth / 2 - .1)

    eps = 0.068
    offset = (kRobotHeight + kBoxHeight) / 2 + eps


    kGrabBoxPositionAngle = getKGrabBoxPositionAngle(offset)

    stacksPos = [pos for pos, _ in getKGrabBoxPositionAngle(0)]
    uniqueStacksPos = makeUnique(stacksPos)

    stacksPartitionMap = partition([2 + i for i in range(len(kGrabBoxPositionAngle))], lambda i: uniqueStacksPos.index(stacksPos[i - 2]) + 2)

    kAtticsPosition = getKAtticsPosition(offset)

    atticsPos = [pos for pos, _ in getKAtticsPosition(0)]
    uniqueAtticsPos = makeUnique(atticsPos)

    startInd = 2 + len(kGrabBoxPositionAngle)
    atticsPartitionMap = partition([startInd + i for i in range(len(kAtticsPosition))], lambda i: uniqueAtticsPos.index(atticsPos[i - startInd]) + startInd)


    n = 1 << 6
    r = np.random.randint(2**32 - 1)
    np.random.seed(2026)
    points = np.array(list(map(list, [nid] * 2 + [pos for pos, _ in kGrabBoxPositionAngle + kAtticsPosition] + ((uniqueStacksPos + uniqueAtticsPos) if addUseless else []))))
    angles = [0.] * 2 + [alpha for _, alpha in kGrabBoxPositionAngle + kAtticsPosition] + ([0.] * (len(uniqueStacksPos) + len(uniqueAtticsPos)) if addUseless else [])
    n = len(points)
    np.random.seed(r)

    dist, distCoefs, angleDist, angleCoefs = calculateDist(points)

    types = [2, 3] + [0] * len(kGrabBoxPositionAngle) + [1] * len(kAtticsPosition) + (([4] * len(uniqueStacksPos) + [5] * len(uniqueAtticsPos)) if addUseless else [])

    # 0 -> récup
    # 1 -> dépôt
    # 2 -> robot
    # 3 -> nid
    # 4 -> stack
    # 5 -> dépôt

    robotState = ACORobotState(
        cost=4e2 + 1e3 * types.count(1),
        types=types,
        partitionMap=stacksPartitionMap | atticsPartitionMap,
        usedPoints=set()
    )

    def evalRoute(route: List[int]):
        state = robotState.copy()

        for e in route:
            state.run(e)

        return 1 + state.cost + sum(dist[route[i], route[i + 1]] for i in range(len(route) - 1)) #+ .1 * sum(angleDist[route[i], route[i + 1], route[i + 2]] for i in range(len(route) - 2))

    def heuristic (pheromone: float, visited: list[int], j: int):
        pheromoneCoef = pheromone
        distCoef = distCoefs[visited[-1], j]
        angleCoef = 1# if len(visited) < 2 else angleCoefs[visited[-2], visited[-1], j]

        colorCoef = 1 # colors[visited[-1]] != colors[j]

        return pheromoneCoef * distCoef * angleCoef * colorCoef

    antCount = 64
    endPoints = ([0], [1])
    explorationRate = .03
    stopProbability = .01
    evaporation = 0.3

    def main():
        nonlocal points
        nonlocal dist, distCoefs, angleDist, angleCoefs

        colorMap = [(0, 1, 0), (0, 0, 1), (0, .7, .5), (0, 0, 0), (.35, .7, .25), (.45, .55, .85)]
        colors = [colorMap[t] for t in types]
        names = { 0: 'Robot (futur winner de la cdf 2026)', 1: 'Nid (la casa du robot)' }

        aco = VisualACO(
            points=points,
            colors=colors,
            showPheromones=False,
            evaporation=evaporation,
            names=names,
            n=n,
            heuristic=heuristic,
            antCount=antCount,
            endPoints=endPoints,
            explorationRate=explorationRate,
            stopProbability=stopProbability,
            evalRoute=evalRoute
        )


        for x, y in uniqueStacksPos: aco.ax.scatter(x, y, color=colorMap[4])
        for x, y in uniqueAtticsPos: aco.ax.scatter(x, y, color=colorMap[5])

        stackArrows: list[Optional[Annotation]] = [aco.ax.annotate("", xytext=pos, xy=pos + offset * getVecAtAngle(angle), arrowprops=dict(arrowstyle="->", color=(0, 1, 0))) for pos, angle in kGrabBoxPositionAngle]
        atticsArrows: list[Optional[Annotation]] = [aco.ax.annotate("", xytext=pos, xy=pos + offset * getVecAtAngle(angle), arrowprops=dict(arrowstyle="->", color=(0, 0, 1))) for pos, angle in kAtticsPosition]

        pointsSpeed = np.random.randn(*points.shape) / 1_000
        pointsSpeed[0] = 0
        pointsSpeed[1] = 0
        pointsSpeed = 0
        robotSpeed = .03

        for _ in aco:
            if aco.inactivityCounter == 16:
                break

        for iteration, bestRoute in enumerate(aco):
            msg = f"Iteration {iteration}: Best cost = {aco.bestCost:.4f} - Inactivity level = {aco.inactivityCounter} - Kaplas in inventory = {robotState.kaplas} - Robot points = {robotState.points}"
            print(msg)
            aco.print(msg)

            dx = robotSpeed
            for i in range(1, len(bestRoute)):
                nextStop = bestRoute[i]
                dir = points[nextStop] - points[0]
                norm = np.linalg.norm(dir)

                ddx = min(dx, norm)
                dx -= ddx
                points[0] += dir / norm * ddx

                if not dx:
                    break

                robotState.run(nextStop)
                aco.scatter.set_alpha([e not in robotState.usedPoints for e in range(n)])

                for e in stacksPartitionMap.get(nextStop, {nextStop}):
                    aco.textNodes[e].set_alpha(0)

                    grabNodeIndex = e - 2
                    if 0 <= grabNodeIndex < len(stackArrows):
                        arrow = stackArrows[grabNodeIndex]
                        if arrow:
                            arrow.remove()
                            # stacksPartitionMap
                            stackArrows[grabNodeIndex] = None
                    
                for e in atticsPartitionMap.get(nextStop, {nextStop}):
                    aco.textNodes[e].set_alpha(0)

                    atticNodeIndex = grabNodeIndex - len(kGrabBoxPositionAngle)
                    if 0 <= atticNodeIndex < len(atticsArrows):
                        arrow = atticsArrows[atticNodeIndex]
                        if arrow:
                            arrow.remove()
                            atticsArrows[atticNodeIndex] = None

                    for (i, j), line in list(aco.edgeLines.items()):
                        if i == e or j == e:
                            line.remove()
                            del aco.edgeLines[i, j]

                    aco.updateBestRoute([ 0 ] + bestRoute[2:])

            points += pointsSpeed
            dist, distCoefs, angleDist, angleCoefs = calculateDist(points)
            aco.refresh()


            # if aco.inactivityCounter >= 4:
            #     for i in range(n):
            #         j = (i + 1) % n

            #         a, b = aco.bestRoute[i], aco.bestRoute[j]

            #         aco.dist[a, b] += 1_024
            #         aco.dist[b, a] += 1_024

            #         aco.ax.plot(
            #             [aco.points[a][0], aco.points[b][0]],
            #             [aco.points[a][1], aco.points[b][1]],
            #             color=(0, 0, 0, 1.0),
            #             linewidth=1
            #         )

            #     aco.updateBestRoute()

            #     resets += 1
            #     print(resets)

    def getACO(team: Literal['blue'] | Literal['yellow'], initialInactivity=16, otherInactivity=16):
        aco = ACO(
            evaporation=evaporation,
            n=n,
            heuristic=heuristic,
            antCount=antCount,
            endPoints=endPoints,
            explorationRate=explorationRate,
            stopProbability=stopProbability,
            evalRoute=evalRoute
        )

        if team == 'yellow':
            nonlocal nid
            nid = (kWorldWidth - nid[0], nid[1])

        points[0] = points[1] = nid

        def generator ():
            inInitialisation = True

            for ids in aco:
                if aco.inactivityCounter < (initialInactivity if inInitialisation else otherInactivity):
                    inInitialisation = False
                    continue

                yield (ids, [(points[i], angles[i]) for i in ids])

        def refreshACO(deleteFirst = False):
            nonlocal dist, distCoefs, angleDist, angleCoefs

            dist, distCoefs, angleDist, angleCoefs = calculateDist(points)

            aco.inactivityCounter = 0
            
            if deleteFirst:
                aco.updateBestRoute([0] + aco.bestRoute[2:])
            else:
                aco.updateBestRoute()

        def updateRobotPos (newPos: np.ndarray):
            points[0] = newPos

        return (generator(), refreshACO, updateRobotPos)


    if __name__ == "__main__":
        from matplotlib import pyplot as plt

        aco, refreshACO, updateRobotPOS = getACO(team='blue', initialInactivity=128, otherInactivity=64)
        ids, path = next(aco)
        fig, ax = plt.subplots(figsize=(10, 6))

        full_line, = ax.plot(
            [points[i,0] for i in ids],
            [points[i,1] for i in ids],
            color=(1, 0, 0, 0.25),
            linewidth=1.5
        )

        def updateACO ():
            nonlocal ids, path
            ids, path = next(aco)

            full_line.set_data(
                [points[i,0] for i in ids],
                [points[i,1] for i in ids]
            )


        colorMap = [
            (0, 1, 0), (0, 0, 1), (0, .7, .5),
            (0, 0, 0), (.35, .7, .25), (.45, .55, .85)
        ]
        colors = [colorMap[t] for t in types]

        plt.ion()

        scat = ax.scatter(points[:, 0], points[:, 1], color=colors)

        stackArrows = [
            ax.annotate("", xytext=pos, xy=pos + offset * getVecAtAngle(angle),
                        arrowprops=dict(arrowstyle="->", color=(0, 1, 0)))
            for pos, angle in kGrabBoxPositionAngle
        ]

        atticArrows = [
            ax.annotate("", xytext=pos, xy=pos + offset * getVecAtAngle(angle),
                        arrowprops=dict(arrowstyle="->", color=(0, 0, 1)))
            for pos, angle in kAtticsPosition
        ]

        progress, = ax.plot([points[1,0]], [points[1,1]], color=(0, 0, 0, 1), linewidth=2)

        robot_dot, = ax.plot([], [], 'o', color=(0, 0, 0))

        last_to_robot, = ax.plot(
            [points[1,0], points[0,0]],
            [points[1,1], points[0,1]],
            color=(0, 0, 0),
            linewidth=2
        )

        robotSpeed = 0.03

        visited = [1]
        visitedSet = set()

        while len(ids) > 1:
            target_id = ids[1]
            target = points[target_id]

            direction = target - points[0]
            norm = np.linalg.norm(direction)

            if norm < 1e-3:
                robotState.run(target_id)
                refreshACO(True)
                updateACO()

                visited.append(target_id)
                visitedSet |= (atticsPartitionMap | stacksPartitionMap).get(target_id, { target_id })

                scat.set_alpha([ e not in visitedSet for e in range(len(points)) ])
                continue


            step = min(robotSpeed, norm)
            updateRobotPOS(points[0] + direction / norm * step)
            refreshACO()

            if visited:
                progress.set_data([points[j,0] for j in [1] + visited[1:]], [points[j,1] for j in [1] + visited[1:]])
                last = visited[-1]
            else:
                last = 1

            robot_dot.set_data([points[0,0]], [points[0,1]])

            last_to_robot.set_data([points[last,0], points[0,0]], [points[last,1], points[0,1]])

            plt.pause(0.1)

        plt.ioff()
        plt.show()

    return getACO



if __name__ == "__main__":
    acoInit()