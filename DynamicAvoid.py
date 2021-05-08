# -*- coding: utf-8 -*-
from dronekit import *
import math
import sys
from PathProgramming import *


def CalculateDist(Location1,Location2):
    Dist = math.sqrt(math.pow(Location1.lat - Location2.lat, 2) * 111700 + math.pow((Location1.lon - Location2.lon) * 111700 * math.cos(round(Location1.lat,1)/180*math.pi), 2))
    return Dist


def CreateTurningPoint(NoflyZones, UAVRadius):
    NoflyZones_Ex = []

    for ZoneNum in range(len(NoflyZones)):
        NoflyZones_Ex.append([])

        pclat = sum([x.lat for x in NoflyZones[ZoneNum]]) / len(NoflyZones[ZoneNum])
        pclng = sum([x.lon for x in NoflyZones[ZoneNum]]) / len(NoflyZones[ZoneNum])
        for i in range(len(NoflyZones[ZoneNum])):
            distHypotenuse = CalculationFormula().CalculateDistance(NoflyZones[ZoneNum][i].lat,
                                                                    NoflyZones[ZoneNum][i].lon, 0, pclat, pclng, 0)
            exnofly_gain = (distHypotenuse + UAVRadius * 2) / distHypotenuse
            NoflyZones_Ex[-1].append(LocationGlobalRelative(pclat + exnofly_gain * (NoflyZones[ZoneNum][i].lat - pclat),
                                     pclng + exnofly_gain * (NoflyZones[ZoneNum][i].lon - pclng),
                                     NoflyZones[ZoneNum][i].alt))

    return NoflyZones_Ex

def CrossOrNot(FromPoint, ToPoint, Radius, NoFlyZone):
    FromPointAdd90, FromPointRedu90, ToPointAdd90, ToPointRedu90 = GenerateTool.GenerateEnvelope(FromPoint, ToPoint,Radius)
    Cross = False
    for From in range(len(NoFlyZone)):
        to = From + 1
        if From >= len(NoFlyZone) - 1:
            to = 0
        Cross |= CalculationFormula.CrossLine(FromPoint.lat, FromPoint.lon, ToPoint.lat, ToPoint.lon,
                                                  NoFlyZone[From].lat,
                                                  NoFlyZone[From].lon, NoFlyZone[to].lat,
                                                  NoFlyZone[to].lon)
        Cross |= CalculationFormula.CrossLine(FromPointAdd90.lat, FromPointAdd90.lon, ToPointAdd90.lat,
                                                  ToPointAdd90.lon, NoFlyZone[From].lat,
                                                  NoFlyZone[From].lon, NoFlyZone[to].lat,
                                                  NoFlyZone[to].lon)
        Cross |= CalculationFormula.CrossLine(FromPointRedu90.lat, FromPointRedu90.lon, ToPointRedu90.lat,
                                                  ToPointRedu90.lon, NoFlyZone[From].lat,
                                                  NoFlyZone[From].lon, NoFlyZone[to].lat,
                                                  NoFlyZone[to].lon)
    return Cross


def ObstacleExpand(Obstacle, UAVRadius):
    Obstacle_Ex = []
    pclat = sum([x.lat for x in Obstacle]) / len(Obstacle)
    pclng = sum([x.lon for x in Obstacle]) / len(Obstacle)
    for i in range(len(Obstacle)):
        distHypotenuse = CalculationFormula.CalculateDistance(Obstacle[i].lat,
                                                              Obstacle[i].lon, 0, pclat, pclng, 0)
        exnofly_gain = (distHypotenuse + UAVRadius * 4) / distHypotenuse
        Obstacle_Ex.append(LocationGlobalRelative(pclat + exnofly_gain * (Obstacle[i].lat - pclat),
                                                  pclng + exnofly_gain * (Obstacle[i].lon - pclng),
                                                  Obstacle[i].alt))
    return Obstacle_Ex

class DStarLite:

    class Point:
        def __init__(self,Location):
            self.loc = Location
            self.Key = 0

    @staticmethod
    def CalculateKey(s,From,To):
        h = CalculationFormula.HaversineFormula(s,From)
        g = CalculationFormula.HaversineFormula(s,To)
        return g + h

    def __init__(self,Start,Goal,NoFlyZones,UAVRadius):

        self.Start = Start
        self.Goal = Goal

        self.UAVRadius = UAVRadius

        self.NoFlyZones = NoFlyZones
        self.NoFlyZones_Ex = CreateTurningPoint(self.NoFlyZones,self.UAVRadius)

        self.Near_NFZ = None

        self.Obstacle = []
        self.Succ_Obs = None

        self.S = []

        self.U = []
#----------------------------------------------------------------------
    def initial(self):

        self.S = []
        self.U = []
        self.Obstacle = []
        self.Succ_Obs = None

    def ComputeShortestPath(self):

        if self.NoFlyZones != []:
            Cross = False
            self.NoFlyZones.sort(key=lambda noflyzone:min([CalculateDist(point,self.Start) for point in noflyzone]))
            for NoFlyZone in self.NoFlyZones:
                for s in self.S:
                    if CrossOrNot(s.loc,self.Goal,self.UAVRadius,NoFlyZone) or CrossOrNot(s.loc,self.S[0].loc,self.UAVRadius,NoFlyZone):
                        Cross = True
                        self.Near_NFZ = NoFlyZone
                        NFZ_index = self.NoFlyZones.index(NoFlyZone)
                        self.S.extend([DStarLite.Point(p) for p in self.NoFlyZones_Ex[NFZ_index]])
                        break

                if Cross == True:
                    break

        next = DStarLite.Point(self.Goal)
        while len(self.S) > 1:
            for s in self.S:
                if CalculationFormula.CrossLine(s.loc.lat,s.loc.lon,next.loc.lat,next.loc.lon,self.Succ_Obs[0].lat,self.Succ_Obs[0].lon,self.Succ_Obs[1].lat,self.Succ_Obs[1].lon):
                    s.Key = sys.float_info.max
                elif self.Near_NFZ != None:
                    if CrossOrNot(s.loc,next.loc,self.UAVRadius,self.Near_NFZ):
                        s.Key = sys.float_info.max
                    else:
                        s.Key = DStarLite.CalculateKey(s.loc,self.Goal,self.S[0].loc)
            next = min(self.S,key=lambda t:t.Key)
            if next == self.S[0]:
                break
            else:
                self.U.append(next.loc)
                self.S.remove(next)
        self.U.reverse()

    def ScanForObstacles(self):
        block = False
        if self.Obstacle.__len__() == 0:
            block |= False
            return block
        else:
            for obs in self.Obstacle:
                if CalculationFormula.CrossLine(self.Start.lat,self.Start.lon,self.Goal.lat,self.Goal.lon,obs[0].lat,obs[0].lon,obs[1].lat,obs[1].lon):
                    self.S.append(DStarLite.Point(self.Start))
                    block = True

                    Obs_EX = ObstacleExpand(obs,self.UAVRadius)
                    self.S.extend([DStarLite.Point(o) for o in Obs_EX])

                    self.Succ_Obs = obs
                    break
            self.Obstacle = []

        return block
