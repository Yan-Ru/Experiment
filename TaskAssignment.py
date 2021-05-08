from dronekit import *
from PathProgramming import *


class TaskAssignment:
    stopiteration = 100
    tabulistlength = 7

    @staticmethod
    def Search(StartPointList,GoalPointList,NoFlyZones,UAVRadiusList):

        ExNoflyPos = GenerateTool.CreateTurningPoint(NoFlyZones,UAVRadiusList)
        distarray, Save_Astar_Ans =  TaskAssignment.CulDistArray(StartPointList, GoalPointList, NoFlyZones, ExNoflyPos, UAVRadiusList)
        Result = TaskAssignment.TabuSearch(len(StartPointList),len(GoalPointList),distarray,TaskAssignment.stopiteration,TaskAssignment.tabulistlength)
        AnsPointList = []
        for i in range(len(StartPointList)):
            Target = next((index for (index, d) in enumerate(Result) if d == i), None)
            AUVLocation = []
            AUVLocation.append(StartPointList[i])
            Ans = next((x for x in Save_Astar_Ans if x.From == i and  x.to == Target), None)
            if Ans != None:
                for point in Ans.ans:
                    AUVLocation.append(point)
            AUVLocation.append(GoalPointList[Target])
            AnsPointList.append(AUVLocation)
        return AnsPointList

    @staticmethod
    def CulDistArray(StartPos,GoalPos,NoFlyZones,NoFlyZones_EX,UAVRadiusList):
        Save_Astar_Ans = []
        distanceArray = [[0 for _ in range(len(GoalPos))] for _ in range(len(StartPos))]
        for i in range(len(StartPos)):
            for j in range(len(GoalPos)):
                if CalculationFormula.IfCrossNoFlyZone(StartPos[i],GoalPos[j],UAVRadiusList[i],NoFlyZones) == False:
                    dist = CalculationFormula.CalculateDistance(StartPos[i].lat, StartPos[i].lon, StartPos[i].alt, GoalPos[j].lat, GoalPos[j].lon, GoalPos[j].alt)
                    distanceArray[i][j] = dist
                else:
                    Save_Astar_Ans.append(AStar.Astar_Ans(From=i,to=j))
                    distanceArray[i][j],Save_Astar_Ans[-1].ans = AStar.Search(StartPos[i],GoalPos[j],NoFlyZones,NoFlyZones_EX[i],UAVRadiusList[i],Save_Astar_Ans[-1].ans)
        return distanceArray, Save_Astar_Ans

    @staticmethod
    def TabuSearch(StartPointCount,GoalPointCount,distarray,StopIteration,TabuListLength):
        Optimal = []
        for i in range(StartPointCount):
            Optimal.append(i)
        OptimalValue = [0 for _ in range(2)]
        for k in range(GoalPointCount):
            OptimalValue[1] += distarray[Optimal[k]][k]
            if OptimalValue[0] < distarray[Optimal[k]][k]:
                OptimalValue[0] = distarray[Optimal[k]][k]
        Current = []
        for i in range(StartPointCount):
            Current.append(Optimal[i])
        TabuList = [[0 for _ in range(2)]for _ in range(TabuListLength)]
        TabuListNum = 0
        K = 0
        while True:
            NearOptimal = []
            for i in range(StartPointCount):
                NearOptimal.append(Current[i])
            NearOptimalValue = [sys.float_info.max for _ in range(2)]
            for i in range(StartPointCount):
                for j in range(i+1,StartPointCount):
                    a = Current[i]
                    b = Current[j]
                    move = []
                    for k in range(StartPointCount):
                        move.append(Current[k])
                    move[i] = b
                    move[j] = a
                    NearValue = [0 for _ in range(2)]
                    for k in range(GoalPointCount):
                        NearValue[1] += distarray[move[k]][k]
                        if NearValue[0] < distarray[move[k]][k]:
                            NearValue[0] = distarray[move[k]][k]
                    ok = True
                    for k in range(TabuListLength):
                        ok &= not(TabuList[k][0] == NearValue[0] and TabuList[k][1] == NearValue[1])
                    if ok:
                        if NearValue[1] < NearOptimalValue[1]:
                            NearOptimalValue[0] = NearValue[0]
                            NearOptimalValue[1] = NearValue[1]
                            TabuList[TabuListNum][0] = NearValue[0]
                            TabuList[TabuListNum][1] = NearValue[1]
                            for k in range(StartPointCount):
                                NearOptimal[k] = move[k]
            if TabuListNum == TabuListLength - 1:
                TabuListNum = 0
            else:
                TabuListNum += 1
            for i in range(StartPointCount):
                Current[i] = NearOptimal[i]
            if NearOptimalValue[1] < OptimalValue[1]:
                OptimalValue[0] = NearOptimalValue[0]
                OptimalValue[1] = NearOptimalValue[1]
                for k in range(StartPointCount):
                    Optimal[k] = NearOptimal[k]
            K += 1
            if K == StopIteration:
                break
        return Optimal