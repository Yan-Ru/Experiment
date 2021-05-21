import math
import sys

class LocationGlobalRelative:
    def __init__(self,lat,lon,alt=0):
        self.lat = lat
        self.lon = lon
        self.alt = alt

    def __getitem__(self, item):
        return self.__dict__[item]

class mTSP:

    @staticmethod
    def Search(StartPointList,InterestPointList,GoalPointList,NoFlyZones,UAVRadiusList,RouteBalance):
        global RouteBalance_TF
        RouteBalance_TF = RouteBalance
        Pointlist = []
        Save_Astar_Ans = []

        Codition = False

        if (len(StartPointList) == len(GoalPointList)):
            for i in range(len(StartPointList)):
                Codition &= StartPointList[i].lat == GoalPointList[i].lat and StartPointList[i].lon == GoalPointList[i].lon and StartPointList[i].alt == GoalPointList[i].alt

        for i in range(len(StartPointList)):

            Pointlist.append(StartPointList[i])

        for i in range(len(GoalPointList)):
            Pointlist.append(GoalPointList[i])

        for i in range(len(InterestPointList)):
            Pointlist.append(InterestPointList[i])
        global m_num
        m_num = len(UAVRadiusList)
        PointNum = len(Pointlist)
        NoflyZone_Ex_Count = 0

        for noflyzone in NoFlyZones:
            NoflyZone_Ex_Count += len(noflyzone)

        NoflyZones_Ex = GenerateTool.CreateTurningPoint(NoFlyZones, UAVRadiusList)
        DistanceArray,Save_Astar_Ans = mTSP.CreateDistanceArray(Pointlist,NoFlyZones,NoflyZones_Ex,UAVRadiusList,Save_Astar_Ans)

        path_solution = mTSP.improve(Codition,len(StartPointList),len(GoalPointList),len(InterestPointList),DistanceArray)
        AnswerPointList = []
        for i in range(len(path_solution)):
            AnswerPointList.append([])
            for j in range(len(path_solution[i]) - 1):
                Ans = next((x for x in Save_Astar_Ans if x.From == path_solution[i][j] and x.to == path_solution[i][j+1]), None)
                if Ans != None:
                    for point in Ans.ans:
                        AnswerPointList[i].append(point)
                else:
                    AnswerPointList[i].append(Pointlist[path_solution[i][j]])
            AnswerPointList[i].append(Pointlist[path_solution[i][-1]])

        return AnswerPointList

    global final_sol_no_ch,tabulist_length,intabu_no_ch,m_num,iterat,crstabu_no_ch,RouteBalance_TF
    final_sol_no_ch = 3
    tabulist_length = 10
    intabu_no_ch = 30
    m_num = 1
    iterat = 0
    crstabu_no_ch = 30
    RouteBalance_TF = bool

    @staticmethod
    def deepCopy( From ):
        to = []
        for path in From:
            to.append(path[0:len(path)])
        return to

    @staticmethod
    def calCost(multi_path,distanceArray):
        opt_total = 0
        opt_max = 0
        for i in range(len(multi_path)):
            dist_f = mTSP.path_distance(multi_path[i], distanceArray[0])
            opt_total += dist_f
            if dist_f > opt_max:
                opt_max = dist_f
        return opt_max, opt_total

    @staticmethod
    def improve(Codition, StartPointNum, GoalPointNum, TargetPointNum, distanceArray):
        multi_path = mTSP.Initial_mTSP_Path(StartPointNum, GoalPointNum, TargetPointNum)
        opt_path = mTSP.deepCopy(multi_path)
        opt_max,opt_total = mTSP.calCost(opt_path,distanceArray)
        count = 0
        global final_sol_no_ch
        while True:
            multi_path = mTSP.deepCopy(mTSP.Tabu_cross_route(Codition,multi_path,distanceArray))
            global m_num
            for m in range(m_num):
                singlePathImprove = mTSP.Tabu_in_route(multi_path[m],distanceArray[m])
                multi_path[m] = singlePathImprove[0:len(singlePathImprove)]

            cur_max,cur_total = mTSP.calCost(multi_path,distanceArray)

            if mTSP.compare_max_total(cur_max,cur_total,opt_max,opt_total):
                opt_max = cur_max
                opt_total = cur_total
                opt_path = mTSP.deepCopy(multi_path)

                count = 0

            else:
                count += 1
            global iterat
            iterat += 1
            if count == final_sol_no_ch:
                break
        return opt_path

    @staticmethod
    def Tabu_in_route(path,distanceArray):

        current_solution = path[0:len(path)]
        global tabulist_length
        tabulist = [0 for i in range(tabulist_length)]
        L = 0
        sol_opt_path = path[0:len(path)]

        opt_path_d = mTSP.path_distance(sol_opt_path,distanceArray)
        no_change = 0
        global intabu_no_ch
        while True:
            near_opt_path = list()
            near_opt_d = sys.float_info.max
            Aspiration_path = list()
            Aspiration_d = sys.float_info.max
            for reverseStartIndex in range(1,len(path)-1):
                for reverseCount in range(2,len(path)-reverseStartIndex):
                    path_1 = current_solution[0:len(current_solution)]
                    path_1[reverseStartIndex : reverseCount+reverseStartIndex] = list(reversed(path_1[reverseStartIndex : reverseCount+reverseStartIndex]))
                    path_1_d = mTSP.path_distance(path_1,distanceArray)
                    in_list = False
                    for k in range(len(tabulist)):
                        in_list |= path_1_d == tabulist[k]
                    if path_1_d < near_opt_d and not in_list:
                        near_opt_d = path_1_d
                        near_opt_path = path_1[0:len(path_1)]
                    if path_1_d < Aspiration_d:
                        Aspiration_d = path_1_d
                        Aspiration_path = path_1[0:len(path_1)]
            if len(near_opt_path) == 0 :
                near_opt_path = Aspiration_path[0:len(Aspiration_path)]
            tabulist[L] = near_opt_d
            L += 1
            if L == tabulist_length:
                L = 0
            if near_opt_d < opt_path_d:
                opt_path_d = near_opt_d
                sol_opt_path = near_opt_path[0:len(near_opt_path)]
                no_change = 0
            else:
                no_change += 1
            current_solution = near_opt_path[0:len(near_opt_path)]
            if no_change == intabu_no_ch:
                break
        return sol_opt_path



    @staticmethod
    def Tabu_cross_route(Codition,multi_path,distanceArray):
        current_solution = mTSP.deepCopy(multi_path)

        tabulist = [[0 for _ in range(2)] for _ in range(tabulist_length)]
        tabu_cnt = 0
        global_opt_path = mTSP.deepCopy(current_solution)

        global_opt_max, global_opt_total = mTSP.calCost(global_opt_path, distanceArray)

        global_sub_path = mTSP.deepCopy(current_solution)
        global_sub_max = sys.float_info.max
        global_sub_total = sys.float_info.max

        initial_max, initial_total = mTSP.calCost(multi_path, distanceArray)
        count = 0
        while True:
            near_opt_path = []
            near_opt_max = sys.float_info.max
            near_opt_total = sys.float_info.max

            for fstGroupIndex in range(m_num - 1):
                for secGroupIndex in range(fstGroupIndex + 1, m_num - fstGroupIndex):
                    for fstSideNum in range(1, len(current_solution[fstGroupIndex])):
                        first_cut_head_path = current_solution[fstGroupIndex][0:fstSideNum]
                        first_cut_tail_path = current_solution[fstGroupIndex][
                                              fstSideNum:len(current_solution[fstGroupIndex])]

                        for secSideNum in range(1, len(current_solution[secGroupIndex])):
                            second_cut_head_path = current_solution[secGroupIndex][0:secSideNum]
                            second_cut_tail_path = current_solution[secGroupIndex][
                                                   secSideNum:len(current_solution[secGroupIndex])]
                            if Codition:
                                multi_path_forward = mTSP.deepCopy(current_solution)
                                multi_path_forward[fstGroupIndex] = []
                                multi_path_forward[fstGroupIndex].extend(first_cut_head_path)
                                multi_path_forward[fstGroupIndex].extend(second_cut_tail_path)
                                multi_path_forward[secGroupIndex] = []
                                multi_path_forward[secGroupIndex].extend(second_cut_head_path)
                                multi_path_forward[secGroupIndex].extend(first_cut_tail_path)

                                multi_path_reverse = mTSP.deepCopy(current_solution)
                                multi_path_reverse[fstGroupIndex] = []
                                multi_path_reverse[fstGroupIndex].extend(first_cut_head_path)
                                for i in range(len(second_cut_tail_path) - 1, -1, -1):
                                    multi_path_reverse[fstGroupIndex].append(second_cut_head_path[i])
                                multi_path_reverse[secGroupIndex] = []
                                for i in range(len(second_cut_tail_path) - 1, -1, -1):
                                    multi_path_reverse[secGroupIndex].append(second_cut_tail_path[i])
                                multi_path_reverse[secGroupIndex].extend(first_cut_tail_path)

                                max_dist_f = 0
                                total_dist_f = 0
                                max_dist_r = 0
                                total_dist_r = 0
                                not_feasible_f = False
                                not_feasible_r = False
                                for m in range(m_num):
                                    dist_f = mTSP.path_distance(multi_path_forward[m], distanceArray[m])
                                    dist_r = mTSP.path_distance(multi_path_reverse[m], distanceArray[m])
                                    if dist_f == 0:
                                        not_feasible_f = True
                                    if dist_r == 0:
                                        not_feasible_r = True
                                    total_dist_f += dist_f
                                    total_dist_r += dist_r
                                    if (dist_f > max_dist_f) and RouteBalance_TF:
                                        max_dist_f = dist_f
                                    if (dist_r > max_dist_r) and RouteBalance_TF:
                                        max_dist_r = dist_r
                                for t in range(tabulist_length):
                                    not_feasible_f |= tabulist[t][0] == max_dist_f and tabulist[t][1] == total_dist_f
                                    not_feasible_r |= tabulist[t][0] == max_dist_r and tabulist[t][1] == total_dist_r

                                if mTSP.compare_max_total(max_dist_f, total_dist_f, max_dist_r, total_dist_r):
                                    if not_feasible_f == True:
                                        if not_feasible_r == True:
                                            pass
                                        else:
                                            if mTSP.compare_max_total(max_dist_r, total_dist_r, near_opt_max,
                                                                      near_opt_total):
                                                near_opt_max = max_dist_r
                                                near_opt_total = total_dist_r
                                                near_opt_path = mTSP.deepCopy(multi_path_reverse)
                                    else:
                                        if mTSP.compare_max_total(max_dist_f, total_dist_f, near_opt_max,
                                                                  near_opt_total):
                                            near_opt_max = max_dist_f
                                            near_opt_total = total_dist_f
                                            near_opt_path = mTSP.deepCopy(multi_path_forward)
                                else:
                                    if not_feasible_r == True:
                                        if not_feasible_f == True:
                                            pass
                                        else:
                                            if mTSP.compare_max_total(max_dist_f, total_dist_f, near_opt_max,
                                                                      near_opt_total):
                                                near_opt_max = max_dist_f
                                                near_opt_total = total_dist_f
                                                near_opt_path = mTSP.deepCopy(multi_path_forward)
                                    else:
                                        if mTSP.compare_max_total(max_dist_r, total_dist_r, near_opt_max,
                                                                  near_opt_total):
                                            near_opt_max = max_dist_r
                                            near_opt_total = total_dist_r
                                            near_opt_path = mTSP.deepCopy(multi_path_reverse)
                            else:
                                multi_path_forward = mTSP.deepCopy(current_solution)
                                multi_path_forward[fstGroupIndex] = []
                                multi_path_forward[fstGroupIndex].extend(first_cut_head_path)
                                multi_path_forward[fstGroupIndex].extend(second_cut_tail_path)
                                multi_path_forward[secGroupIndex] = []
                                multi_path_forward[secGroupIndex].extend(second_cut_head_path)
                                multi_path_forward[secGroupIndex].extend(first_cut_tail_path)

                                max_dist_f = 0
                                total_dist_f = 0
                                not_feasible_f = False
                                for m in range(m_num):
                                    dist_f = mTSP.path_distance(multi_path_forward[m], distanceArray[m])
                                    if dist_f == 0:
                                        not_feasible_f = True
                                    total_dist_f += dist_f
                                    if dist_f > max_dist_f and RouteBalance_TF:
                                        max_dist_f = dist_f

                                for t in range(tabulist_length):
                                    not_feasible_f |= tabulist[t][0] == max_dist_f and tabulist[t][1] == total_dist_f

                                if not_feasible_f == False:
                                    if mTSP.compare_max_total(max_dist_f, total_dist_f, near_opt_max, near_opt_total):
                                        near_opt_max = max_dist_f
                                        near_opt_total = total_dist_f
                                        near_opt_path = mTSP.deepCopy(multi_path_forward)

            current_solution = mTSP.deepCopy(near_opt_path)

            tabulist[tabu_cnt][0] = near_opt_max
            tabulist[tabu_cnt][1] = near_opt_total
            tabu_cnt += 1

            if tabu_cnt == tabulist_length:
                tabu_cnt = 0
            if mTSP.compare_max_total(near_opt_max, near_opt_total, global_opt_max, global_opt_total):
                global_opt_max = near_opt_max
                global_opt_total = near_opt_total
                global_opt_path = mTSP.deepCopy(near_opt_path)
                count = 0
            else:
                count += 1
                if mTSP.compare_max_total(near_opt_max, near_opt_total, global_sub_max, global_sub_total) and (
                        near_opt_max != global_opt_max or near_opt_total != global_opt_total):
                    global_sub_max = near_opt_max
                    global_sub_total = near_opt_total
                    global_sub_path = mTSP.deepCopy(near_opt_path)

            if count == crstabu_no_ch:
                break
        if iterat >= 1:
            if global_opt_max == initial_max and global_opt_total == initial_total:
                global_opt_path = mTSP.deepCopy(global_sub_path)

        return global_opt_path

    @staticmethod
    def compare_max_total(a_max,a_total,b_max,b_total):
        ask_tf = False
        if a_max < b_max:
            ask_tf = True
        else:
            if a_max == b_max:
                if a_total < b_total:
                    ask_tf = True
                else:
                    ask_tf = False
        return ask_tf


    @staticmethod
    def path_distance(path,Cij):
        path_dist = 0
        for i in range(len(path)-1):
            value = Cij[path[i]][path[i+1]]
            if value >= sys.float_info.max:
                value = 0
            path_dist += value
        if path_dist == 0:
            path_dist = sys.float_info.max
        return path_dist

    @staticmethod
    def CreateDistanceArray(PointList, NoflyZones, NoflyZones_Ex, UAVRadiusList, Save_Astar_Ans):

        global m_num
        DistanceArray = [[[0 for _ in range(len(PointList))] for _ in range(len(PointList))]for _ in range(m_num)]
        for i in range(len(PointList)):
            for j in range(len(PointList)):
                for k in range(m_num):
                    if CalculationFormula.IfCrossNoFlyZone(PointList[i],PointList[j],UAVRadiusList[k],NoflyZones) == False:
                        DistanceArray[k][i][j] = CalculationFormula.CalculateDistance(PointList[i].lat,PointList[i].lon,PointList[i].alt,PointList[j].lat,PointList[j].lon,PointList[j].alt)

                    else:
                        Save_Astar_Ans.append(AStar.Astar_Ans(From=i,to=j))
                        DistanceArray[k][i][j],Save_Astar_Ans[-1].ans = AStar.Search(PointList[i],PointList[j],NoflyZones,NoflyZones_Ex[k],UAVRadiusList[k],Save_Astar_Ans[-1].ans)

        return DistanceArray,Save_Astar_Ans


    @staticmethod
    def Initial_mTSP_Path(StartPointNum, GoalPointNum, TargetPointNum):
        path_initial = []
        for i in range(m_num):
            path_initial.append([])
            if i <= StartPointNum - 1:
                path_initial[i].append(i)
            else:
                path_initial[i].append(StartPointNum - 1)

            AddTargetNum = TargetPointNum / m_num
            if (i < TargetPointNum % m_num):
                AddTargetNum += 1

            for j in range(int(AddTargetNum)):
                path_initial[i].append(StartPointNum + GoalPointNum + i + j * m_num)

            if i <= GoalPointNum - 1:
                path_initial[i].append(StartPointNum + i)
            else:
                path_initial[i].append(StartPointNum - 1 + GoalPointNum)

        return path_initial


class AStar:
    class Astar_Ans:
        def __init__(self,From=0,to=0):
            self.From = From
            self.to = to
            self.ans = []

        def __getitem__(self, item):
            return self.__dict__[item]


    class AStarPoint:

        def __init__(self,ThisPoint,FromPoint,TargetPoint,GValuePast,DistanceArray,HValueArray):
            self.ThisPoint = ThisPoint
            self.FromPoint = FromPoint
            self.GValue = GValuePast + DistanceArray[ThisPoint][FromPoint]
            self.FValue = self.GValue + HValueArray[ThisPoint]
        def __getitem__(self, item):
            return self.__dict__[item]

    @staticmethod
    def Search(startPoint,targetPoint,NoFlyZones,NoFlyZones_Ex,UAVRadius,AnswerPointList):
        PathDistance = 0
        Open = []
        Close = []
        DE = []
        PointList = []
        PointList.append(startPoint)
        for i in range(len(NoFlyZones_Ex)):
            PointList.append(LocationGlobalRelative(NoFlyZones_Ex[i].lat,NoFlyZones_Ex[i].lon,0))
        PointList.append(LocationGlobalRelative(targetPoint.lat,targetPoint.lon,0))
        t = [0 for _ in range(len(NoFlyZones_Ex)+2)]
        for i in range(len(PointList)):
            t[i]= LocationGlobalRelative(PointList[i].lat,PointList[i].lon,0)
        DistanceArray = AStar.smalldistance(t,NoFlyZones,UAVRadius)
        HValueArray = [0 for i in range(len(NoFlyZones_Ex)+2)]
        for i in range(len(HValueArray)):
            HValueArray[i] = CalculationFormula.CalculateDistance(PointList[i].lat,PointList[i].lon,0,targetPoint.lat,targetPoint.lon,0)
        Result = list()
        Open.append(AStar.AStarPoint(0,0,len(NoFlyZones_Ex)+1,0,DistanceArray,HValueArray))
        Open[0].GValue = 0
        Open[0].FValue = 0
        while (len(Open) != 0):

            minPoint = min(Open,key=lambda t:t.FValue)
            CurrentPoint = minPoint
            Open.remove(minPoint)

            Expand = []
            InLoop = False
            for expandPoint in range(len(NoFlyZones_Ex)+2):
                if DistanceArray[expandPoint][CurrentPoint.ThisPoint] != sys.float_info.max:
                    if next((x for x in DE if x.ThisPoint == expandPoint), None) == None:
                        if next((y for y in Close if y.ThisPoint == expandPoint), None) == None:
                            ExpandPoint = AStar.AStarPoint(expandPoint,CurrentPoint.ThisPoint,len(NoFlyZones_Ex)+1,CurrentPoint.GValue,DistanceArray,HValueArray)
                            LoopPointIndex = next((index for (index, d) in enumerate(Open) if d["ThisPoint"] == expandPoint), None)
                            if LoopPointIndex == None:
                                Expand.append(ExpandPoint)
                            elif Open[LoopPointIndex].FValue > ExpandPoint.FValue:
                                InLoop = True
                                Open[LoopPointIndex] = ExpandPoint
            if (len(Expand) != 0) or (InLoop == True):
                Close.append(CurrentPoint)
                for expand in Expand:
                    Open.append(expand)
            else:
                DE.append(CurrentPoint)

            FindTargetPoint = next((t for t in Open if t.ThisPoint == len(NoFlyZones_Ex)+1), None)
            if FindTargetPoint != None:
                PathDistance = FindTargetPoint.GValue
                Result.append(len(NoFlyZones_Ex)+1)
                Result.append(CurrentPoint.ThisPoint)
                Result.append(CurrentPoint.FromPoint)
                if Result[-1] != 0:
                    while True:
                        now = Result[-1]
                        select =  next((x for x in Close if x.ThisPoint == now))
                        Result.append(select.FromPoint)
                        if Result[-1] == 0:
                            break
                AnswerPointList.append(startPoint)
                d = 0
                for i in range(len(Result)-2,0,-1):
                    AnswerPointList.append(PointList[Result[i]])
                    d += DistanceArray[Result[i+1]][Result[i]]
                    AnswerPointList[-1].alt = startPoint.alt + (targetPoint.alt - startPoint.alt) * (d / PathDistance)
                AnswerPointList.append(targetPoint)
                PathDistance = math.sqrt(math.pow(PathDistance,2) + math.pow(targetPoint.alt - startPoint.alt,2))
                break
        return PathDistance,AnswerPointList

    @staticmethod
    def smalldistance(s,NoflyZones,r):
        Cij = [[0 for _ in range(len(s))] for _ in range(len(s))]
        for i in range(len(s)):
            for j in range(len(s)):
                if CalculationFormula().IfCrossNoFlyZone(s[i],s[j],r,NoflyZones) == False:
                    Cij[i][j] = CalculationFormula().CalculateDistance(s[i].lat,s[i].lon,s[i].alt,s[j].lat,s[j].lon,s[j].alt)
                else:
                    Cij[i][j] = sys.float_info.max
        return Cij


class GenerateTool:

    @staticmethod
    def CreateTurningPoint(NoflyZones,UAVRadiusList):
        NoflyZones_Ex = []
        for UAV_Num in range(len(UAVRadiusList)):
            NoflyZones_Ex.append([])
            for ZoneNum in range(len(NoflyZones)):
                pclat = sum([x.lat for x in NoflyZones[ZoneNum]]) / len(NoflyZones[ZoneNum])
                pclng = sum([x.lon for x in NoflyZones[ZoneNum]]) / len(NoflyZones[ZoneNum])
                for i in range(len(NoflyZones[ZoneNum])):
                    distHypotenuse = CalculationFormula().CalculateDistance(NoflyZones[ZoneNum][i].lat ,NoflyZones[ZoneNum][i].lon ,0 ,pclat ,pclng ,0 )
                    exnofly_gain = (distHypotenuse + UAVRadiusList[UAV_Num] * 2)/distHypotenuse
                    NoflyZones_Ex[-1].append(LocationGlobalRelative(pclat + exnofly_gain * (NoflyZones[ZoneNum][i].lat - pclat), pclng + exnofly_gain * (NoflyZones[ZoneNum][i].lon - pclng),NoflyZones[ZoneNum][i].alt))
                    Point2Index = i + 1
                    if Point2Index == len(NoflyZones[ZoneNum]):
                        Point2Index = 0
                    Heading = math.atan2((NoflyZones[ZoneNum][Point2Index].lon - NoflyZones[ZoneNum][i].lon), (NoflyZones[ZoneNum][Point2Index].lat - NoflyZones[ZoneNum][i].lat))
                    NoflyZones_Ex[-1].append(GenerateTool.GeneratePoint(NoflyZones[ZoneNum][Point2Index], UAVRadiusList[UAV_Num], Heading))
                    NoflyZones_Ex[-1].append(GenerateTool.GeneratePoint(NoflyZones[ZoneNum][Point2Index], UAVRadiusList[UAV_Num], Heading + math.pi / 2))
                    NoflyZones_Ex[-1].append(GenerateTool.GeneratePoint(NoflyZones[ZoneNum][Point2Index], UAVRadiusList[UAV_Num], Heading - math.pi / 2))

        return NoflyZones_Ex

    @staticmethod
    def GenerateEnvelope(FromPoint ,ToPoint ,r):
        Heading = math.atan2((ToPoint.lon - FromPoint.lon),(ToPoint.lat - FromPoint.lat))
        newHeading = Heading + math.pi / 2
        FromPointAdd90 = GenerateTool.GeneratePoint(FromPoint, r, newHeading)
        newHeading = Heading - math.pi / 2
        FromPointRedu90 = GenerateTool.GeneratePoint(FromPoint, r ,newHeading)
        newHeading = Heading + math.pi / 2
        ToPointAdd90 = GenerateTool.GeneratePoint(ToPoint ,r , newHeading)
        newHeading = Heading - math.pi / 2
        ToPointRedu90 = GenerateTool.GeneratePoint(ToPoint ,r ,newHeading)
        return FromPointAdd90,FromPointRedu90,ToPointAdd90,ToPointRedu90

    @staticmethod
    def GenerateSurround(Point ,r ,Num):
        SurroundPoint = list()
        for i in range(Num):
            SurroundPoint.append(GenerateTool.GeneratePoint(Point ,r ,i * ((2 * math.pi) / Num)))
        return SurroundPoint

    @staticmethod
    def GeneratePoint(Point, d , heading):
        return LocationGlobalRelative(Point.lat + d * math.cos(heading) / (6378137 * (math.pi / 180)) , Point.lon + d * math.sin(heading) / (6378137 * math.cos(Point.lat * math.pi / 180) * (math.pi / 180)),Point.alt)

class CalculationFormula:

    @staticmethod
    def HaversineFormula(a, b):
        p = math.pi / 180
        a = 0.5 - math.cos((b.lat - a.lat) * p) / 2 + math.cos(a.lat * p) * math.cos(b.lat * p) * (
                    1 - math.cos((b.lon - a.lon) * p)) / 2
        return 2 * 6371009 * math.asin(math.sqrt(a))

    @staticmethod
    def CalculateDistance(lat0,lng0,alt0,lat1,lng1,alt1):
        if alt0 == None:
            alt0 = 0
        if alt1 == None:
            alt1 = 0
        X = math.cos(lat0 * math.pi / 180) * math.cos(lng0 * math.pi / 180) - math.cos(lat1 * math.pi / 180) * math.cos(lng1 * math.pi / 180)
        Y = math.cos(lat0 * math.pi / 180) * math.sin(lng0 * math.pi / 180) - math.cos(lat1 * math.pi / 180) * math.sin(lng1 * math.pi / 180)
        Z = math.sin(lat0 * math.pi / 180) - math.sin(lat1 * math.pi / 180)
        Tij = 2 * 6378137 * math.asin(math.sqrt(math.pow(X,2) + math.pow(Y,2) + math.pow(Z,2))/2)
        Tij = math.sqrt(math.pow(Tij, 2) + math.pow(alt0-alt1, 2))
        if (lat0,lng0,alt0) != (lat1,lng1,alt1) and Tij == 0:
            return sys.float_info.max
        else: return Tij

    @staticmethod
    def IfCrossNoFlyZone(FromPoint,ToPoint,Radius,NoFlyZones):

        FromPointAdd90,FromPointRedu90,ToPointAdd90,ToPointRedu90 = GenerateTool().GenerateEnvelope(FromPoint,ToPoint,Radius)
        Cross = False
        for ZoneNum in range(len(NoFlyZones)):
            for From in range(len(NoFlyZones[ZoneNum])):
                to = From + 1
                if From >= len(NoFlyZones[ZoneNum])-1:
                    to = 0
                Cross = Cross or CalculationFormula().CrossLine(FromPoint.lat,FromPoint.lon,ToPoint.lat,ToPoint.lon,NoFlyZones[ZoneNum][From].lat,NoFlyZones[ZoneNum][From].lon,NoFlyZones[ZoneNum][to].lat,NoFlyZones[ZoneNum][to].lon)
                Cross = Cross or CalculationFormula().CrossLine(FromPointAdd90.lat,FromPointAdd90.lon,ToPointAdd90.lat,ToPointAdd90.lon,NoFlyZones[ZoneNum][From].lat,NoFlyZones[ZoneNum][From].lon,NoFlyZones[ZoneNum][to].lat,NoFlyZones[ZoneNum][to].lon)
                Cross = Cross or CalculationFormula().CrossLine(FromPointRedu90.lat,FromPointRedu90.lon,ToPointRedu90.lat,ToPointRedu90.lon,NoFlyZones[ZoneNum][From].lat,NoFlyZones[ZoneNum][From].lon,NoFlyZones[ZoneNum][to].lat,NoFlyZones[ZoneNum][to].lon)
        return Cross

    @staticmethod
    def IfCrossNoFlyZone3D(FromPoint,ToPoint,Radius,NoflyZones):
        FromPointAdd90,FromPointRedu90,ToPointAdd90,ToPointRedu90 = GenerateTool.GenerateEnvelope(FromPoint,ToPoint,Radius)
        inSegment = False
        for noflyzone in NoflyZones:
            for noflypanel in noflyzone:
                inSegment = CalculationFormula.CrossSurface(FromPoint,ToPoint,noflypanel,inSegment)
                inSegment = CalculationFormula.CrossSurface(FromPointAdd90,ToPointAdd90,noflypanel,inSegment)
                inSegment = CalculationFormula.CrossSurface(FromPointRedu90,ToPointRedu90,noflypanel,inSegment)
        return inSegment

    @staticmethod
    def CrossSurface(p1,p2,Surface,inSegment):
        heading = math.atan2((p2.lon - p1.lon),(p2.lat - p1.lat))
        r = 0.0002
        for o in range(-1,2):
            insegment = True
            delta = heading +((90*o)*math.pi)/180
            P1 = [p1.lat + r * math.cos(delta), p1.lon + r * math.sin(delta), p1.alt]
            P2 = [p2.lat + r * math.cos(delta), p2.lon + r * math.sin(delta), p2.alt]
            V01 = [Surface[0].lat - Surface[1].lat, Surface[0].lon - Surface[1].lon, Surface[0].alt - Surface[1].alt]
            V02 = [Surface[0].lat - Surface[2].lat, Surface[0].lon - Surface[2].lon, Surface[0].alt - Surface[2].alt]
            NV = [V01[1] * V02[2] - V01[2] * V02[1], V01[2] * V02[0] - V01[0] * V02[2], V01[0] * V02[1] - V01[1] * V02[0]]
            K = (NV[0] * P1[0] + NV[1] * P1[1] + NV[2] * P1[2] -(NV[0] * Surface[0].lat + NV[1] * Surface[0].lon + NV[2] * Surface[0].alt)) / (NV[0] * P2[0] - NV[0] * P1[0] + NV[1] * P2[1] - NV[1] * P1[1] + NV[2] * P2[2] - NV[2] * P1[2])
            intersectPoint = [P1[0] + K * P1[0] - K * P2[0], P1[1] + K * P1[1] - K * P2[1], P1[2] + K * P1[2] - K * P2[2]]
            for i in range(3):
                if P2[i] > P1[i]:
                    insegment = insegment and intersectPoint[i] <= P2[i] and intersectPoint[i] >= P1[i]
                else:
                    insegment = insegment and intersectPoint[i] <= P1[i] and intersectPoint[i] >= P2[i]
            if insegment:
                S1 = 0
                for i in range(len(Surface)-1):
                    Va = [(intersectPoint[0] - Surface[i].lat) * 100000, (intersectPoint[1] - Surface[i].lon) * 100000, intersectPoint[2] - Surface[i].alt]
                    Vb = [(intersectPoint[0] - Surface[i + 1].lat) * 100000, (intersectPoint[1] - Surface[i + 1].lon) * 100000, intersectPoint[2] - Surface[i + 1].alt]
                    S1 += math.sqrt(math.pow(Va[1]*Vb[2]-Va[2]*Vb[1],2) + math.pow(Va[2]*Vb[0]-Va[0]*Vb[2],2) + math.pow(Va[0] * Vb[1] - Va[1] * Vb[0], 2)) / 2
                S2 = 0
                for i in range(1,len(Surface)-2):
                    Va = [(Surface[0].lat - Surface[i].lat) * 100000, (Surface[0].lon - Surface[i].lon) * 100000, Surface[0].alt - Surface[i].alt]
                    Vb = [(Surface[0].lat - Surface[i + 1].lat) * 100000, (Surface[0].lon - Surface[i + 1].lon) * 100000, Surface[0].alt - Surface[i + 1].alt]
                    S2 += math.sqrt(math.pow(Va[1] * Vb[2] - Va[2] * Vb[1], 2) + math.pow(Va[2] * Vb[0] - Va[0] * Vb[2], 2) + math.pow(Va[0] * Vb[1] - Va[1] * Vb[0], 2)) / 2
                insegment = insegment and S1 == S2
                inSegment = inSegment or insegment
                inSegment = inSegment or (p2.alt < 0 or p1.alt < 0)
                if inSegment:
                    return inSegment
    @staticmethod
    def CrossLine(lat0,lon0,lat1,lon1,lat2,lon2,lat3,lon3):
        d1 = CalculationFormula.crsproduct(lat2,lon2,lat3,lon3,lat0,lon0)
        d2 = CalculationFormula.crsproduct(lat2,lon2,lat3,lon3,lat1,lon1)
        d3 = CalculationFormula.crsproduct(lat0, lon0, lat1, lon1, lat2, lon2)
        d4 = CalculationFormula.crsproduct(lat0, lon0, lat1, lon1, lat3, lon3)
        if d1 * d2 < 0 and d3 * d4 < 0:
            return True
        elif d1 == 0 and CalculationFormula.sameline(lat2, lon2, lat3, lon3, lat0, lon0):
            return True
        elif d2 == 0 and CalculationFormula.sameline(lat2, lon2, lat3, lon3, lat1, lon1):
            return True
        elif d3 == 0 and CalculationFormula.sameline(lat0, lon0, lat1, lon1, lat2, lon2):
            return True
        elif d4 == 0 and CalculationFormula.sameline(lat0, lon0, lat1, lon1, lat3, lon3):
            return True
        else:return False

    @staticmethod
    def crsproduct(lat0,lon0,lat1,lon1,noflylat0,noflylon0):
        return ((noflylat0-lat0)*(lon1-lon0))-((lat1-lat0)*(noflylon0-lon0))
    @staticmethod
    def sameline(lat0,lon0,lat1,lon1,noflylat0,noflylng0):
        minx = min(lat0,lat1)
        maxx = max(lat0,lat1)
        miny = min(lon0,lon1)
        maxy = max(lon0,lon1)
        return minx <= noflylat0 <= maxx and miny <= noflylng0 <= maxy
